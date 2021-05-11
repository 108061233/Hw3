#include <math.h>
#include "mbed.h"
// MQTT
#include "MQTTNetwork.h"
#include "MQTTmbed.h"
#include "MQTTClient.h"
// Accelerometer
#include "stm32l475e_iot01_accelero.h"
// RPC
#include "mbed_rpc.h"
// lab8
#include "accelerometer_handler.h"
#include "magic_wand_model_data.h"
#include "tensorflow/lite/c/common.h"
#include "tensorflow/lite/micro/kernels/micro_ops.h"
#include "tensorflow/lite/micro/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/version.h"
// LCD
#include "uLCD_4DGL.h"

// GLOBAL VARIABLES
uLCD_4DGL uLCD(D1, D0, D2);
// wifi
WiFiInterface *wifi;
// Interrupt
InterruptIn btn2(USER_BUTTON);
// LED
DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
// RPC
BufferedSerial pc(USBTX, USBRX);
void tilt(Arguments *in, Reply *out);
void UI(Arguments *in, Reply *out);
RPCFunction rpctilt(&tilt, "tilt");
RPCFunction rpcUI(&UI, "UI");
int mode1, mode2;

float angle, angle_dis;
int16_t DataXYZ[3] = {0};
//InterruptIn btn3(SW3);
volatile int message_num = 0;
volatile int arrivedcount = 0;
volatile bool closed = false;

const char* topic = "Mbed";

Thread wifi_thread;
Thread mqtt_thread;
Thread tilt_thread;
Thread UI_thread;
EventQueue queue(32 * EVENTS_EVENT_SIZE);

#define label_num 3
struct Config
{

   // This must be the same as seq_length in the src/model_train/config.py
   const int seq_length = 64;

   // The number of expected consecutive inferences for each gesture type.
   const int consecutiveInferenceThresholds[label_num] = {20, 10, 10};

   const char *output_message[label_num] = {
       "RING:\n\r"
       "          *       \n\r"
       "       *     *    \n\r"
       "     *         *  \n\r"
       "    *           * \n\r"
       "     *         *  \n\r"
       "       *     *    \n\r"
       "          *       \n\r",
       "SLOPE:\n\r"
       "        *        \n\r"
       "       *         \n\r"
       "      *          \n\r"
       "     *           \n\r"
       "    *            \n\r"
       "   *             \n\r"
       "  *              \n\r"
       " * * * * * * * * \n\r",
       "line:\n\r"
       "                 \n\r"
       " * * * * * * * * \n\r"
       "                 \n\r"};
};
Config config;

/*-------------TF set------------*/

// Create an area of memory to use for input, output, and intermediate arrays.
// The size of this will depend on the model you're using, and may need to be
// determined by experimentation.
constexpr int kTensorArenaSize = 60 * 1024;
uint8_t tensor_arena[kTensorArenaSize];

// Return the result of the last prediction
int PredictGesture(float* output) {
  // How many times the most recent gesture has been matched in a row
  static int continuous_count = 0;
  // The result of the last prediction
  static int last_predict = -1;

  // Find whichever output has a probability > 0.8 (they sum to 1)
  int this_predict = -1;
  for (int i = 0; i < label_num; i++) {
    if (output[i] > 0.8) this_predict = i;
  }

  // No gesture was detected above the threshold
  if (this_predict == -1) {
    continuous_count = 0;
    last_predict = label_num;
    return label_num;
  }

  if (last_predict == this_predict) {
    continuous_count += 1;
  } else {
    continuous_count = 0;
  }
  last_predict = this_predict;

  // If we haven't yet had enough consecutive matches for this gesture,
  // report a negative result
  if (continuous_count < config.consecutiveInferenceThresholds[this_predict]) {
    return label_num;
  }
  // Otherwise, we've seen a positive result, so clear all our variables
  // and report it
  continuous_count = 0;
  last_predict = -1;

  return this_predict;
}

void LCD(float angle_dis)
{
    uLCD.text_width(4);
    uLCD.text_height(4);
    uLCD.locate(0,0);
    uLCD.printf("%.2f", angle_dis);
}

void messageArrived(MQTT::MessageData& md) {
    MQTT::Message &message = md.message;
    char msg[300];
    sprintf(msg, "Message arrived: QoS%d, retained %d, dup %d, packetID %d\r\n", message.qos, message.retained, message.dup, message.id);
    printf(msg);
    ThisThread::sleep_for(1000ms);
    char payload[300];
    sprintf(payload, "Payload %.*s\r\n", message.payloadlen, (char*)message.payload);
    printf(payload);
    ++arrivedcount;
}

void publish_sel_angle(MQTT::Client<MQTTNetwork, Countdown>* client) {
    MQTT::Message message;
    char buf[100];
    sprintf(buf, "The bound angle %.2f", angle_dis);
    message.qos = MQTT::QOS0;
    message.retained = false;
    message.dup = false;
    message.payload = (void*) buf;
    message.payloadlen = strlen(buf) + 1;
    int rc = client->publish(topic, message);
}

void tilt(Arguments *in, Reply *out)
{
    // In this scenario, when using RPC delimit the two arguments with a space.
    mode1 = in->getArg<int>();
    char buff[200];
    sprintf(buff, "%d", mode1);
    out->putData(buff);
}

void UI(Arguments *in, Reply *out)
{
    // In this scenario, when using RPC delimit the two arguments with a space.
    mode2 = in->getArg<int>();
    char buff[200];
    sprintf(buff, "%d", mode2);
    out->putData(buff);
}

void tilt_angle(MQTT::Client<MQTTNetwork, Countdown>* client)
{
   while(1) {
      if (mode1) {
        myled2 = !myled2;
        BSP_ACCELERO_Init();
        BSP_ACCELERO_AccGetXYZ(DataXYZ);
        float x = sqrt(DataXYZ[0] * DataXYZ[0]);
        float y = sqrt(DataXYZ[1] * DataXYZ[1]);
        float z = sqrt(DataXYZ[2] * DataXYZ[2]);

        angle = atan(sqrt(x * x + y * y) / z) * 180 / 3.14;

        if (angle > 1.0) myled1 = 0;
        else myled1 = 1;

        if (angle > angle_dis) {
            MQTT::Message message;
            char buff[30];
            sprintf(buff, "angle: %.2f", angle);
            message.qos = MQTT::QOS0;
            message.retained = false;
            message.dup = false;
            message.payload = (void*) buff;
            message.payloadlen = strlen(buff) + 1;
            int rc = client->publish(topic, message);
        }
        LCD(angle);
        ThisThread::sleep_for(100ms);
      } else myled2 = 0;      
   }
}

void close_mqtt() {
    closed = true;
}

int angle_sel()
{
    /*-------------TF set------------*/

    // Whether we should clear the buffer next time we fetch data
    bool should_clear_buffer = false;
    bool got_data = false;

    // The gesture index of the prediction
    int gesture_index;

    // Set up logging.
    static tflite::MicroErrorReporter micro_error_reporter;
    tflite::ErrorReporter* error_reporter = &micro_error_reporter;

    // Map the model into a usable data structure. This doesn't involve any
    // copying or parsing, it's a very lightweight operation.
    const tflite::Model* model = tflite::GetModel(g_magic_wand_model_data);
    if (model->version() != TFLITE_SCHEMA_VERSION) {
        error_reporter->Report(
            "Model provided is schema version %d not equal "
            "to supported version %d.",
            model->version(), TFLITE_SCHEMA_VERSION);
        return -1;
    }

    // Pull in only the operation implementations we need.
    // This relies on a complete list of all the ops needed by this graph.
    // An easier approach is to just use the AllOpsResolver, but this will
    // incur some penalty in code space for op implementations that are not
    // needed by this graph.
    static tflite::MicroOpResolver<6> micro_op_resolver;
    micro_op_resolver.AddBuiltin(
        tflite::BuiltinOperator_DEPTHWISE_CONV_2D,
        tflite::ops::micro::Register_DEPTHWISE_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_MAX_POOL_2D,
                               tflite::ops::micro::Register_MAX_POOL_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_CONV_2D,
                               tflite::ops::micro::Register_CONV_2D());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_FULLY_CONNECTED,
                               tflite::ops::micro::Register_FULLY_CONNECTED());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_SOFTMAX,
                               tflite::ops::micro::Register_SOFTMAX());
    micro_op_resolver.AddBuiltin(tflite::BuiltinOperator_RESHAPE,
                               tflite::ops::micro::Register_RESHAPE(), 1);

    // Build an interpreter to run the model with
    static tflite::MicroInterpreter static_interpreter(
        model, micro_op_resolver, tensor_arena, kTensorArenaSize, error_reporter);
    tflite::MicroInterpreter* interpreter = &static_interpreter;

    // Allocate memory from the tensor_arena for the model's tensors
    interpreter->AllocateTensors();

    // Obtain pointer to the model's input tensor
    TfLiteTensor* model_input = interpreter->input(0);
    if ((model_input->dims->size != 4) || (model_input->dims->data[0] != 1) ||
        (model_input->dims->data[1] != config.seq_length) ||
        (model_input->dims->data[2] != kChannelNumber) ||
        (model_input->type != kTfLiteFloat32)) {
        error_reporter->Report("Bad input tensor parameters in model");
        return -1;
    }

    int input_length = model_input->bytes / sizeof(float);

    TfLiteStatus setup_status = SetupAccelerometer(error_reporter);
    if (setup_status != kTfLiteOk) {
        error_reporter->Report("Set up failed\n");
        return -1;
    }

    error_reporter->Report("Set up successful...\n");

    while (true)
        if (mode2) {
            myled3 = !myled3;
            // Attempt to read new data from the accelerometer
            got_data = ReadAccelerometer(error_reporter, model_input->data.f,
                                 input_length, should_clear_buffer);

            // If there was no new data,
            // don't try to clear the buffer again and wait until next time
            if (!got_data) {
                should_clear_buffer = false;
                continue;
            }

            // Run inference, and report any error
            TfLiteStatus invoke_status = interpreter->Invoke();
            if (invoke_status != kTfLiteOk) {
            error_reporter->Report("Invoke failed on index: %d\n", begin_index);
            continue;
            }

            // Analyze the results to obtain a prediction
            gesture_index = PredictGesture(interpreter->output(0)->data.f);

            // Clear the buffer next time we read data
            should_clear_buffer = gesture_index < label_num;

            // Produce an output
            if (gesture_index < label_num) {
                if (gesture_index == 0) angle_dis = 20;
                else if (gesture_index == 1) angle_dis = 40;
                else if (gesture_index == 2) angle_dis = 60;
                else angle_dis = 0;
                error_reporter->Report(config.output_message[gesture_index]);
            }
            LCD(angle_dis);
            ThisThread::sleep_for(100ms);
        } else myled3 = 0;
}

void wifi_set()
{
    /*--------WIFI--------*/
    wifi = WiFiInterface::get_default_instance();
    if (!wifi)
    {
        printf("ERROR: No WiFiInterface found.\r\n");
        return -1;
    }

    printf("\nConnecting to %s...\r\n", MBED_CONF_APP_WIFI_SSID);
    int ret = wifi->connect(MBED_CONF_APP_WIFI_SSID, MBED_CONF_APP_WIFI_PASSWORD, NSAPI_SECURITY_WPA_WPA2);
    if (ret != 0)
    {
        printf("\nConnection error: %d\r\n", ret);
        return -1;
    }

    NetworkInterface *net = wifi;
    MQTTNetwork mqttNetwork(net);
    MQTT::Client<MQTTNetwork, Countdown> client(mqttNetwork);

    //TODO: revise host to your IP
    const char *host = "192.168.43.210";
    printf("Connecting to TCP network...\r\n");

    SocketAddress sockAddr;
    sockAddr.set_ip_address(host);
    sockAddr.set_port(1883);

    printf("address is %s/%d\r\n", (sockAddr.get_ip_address() ? sockAddr.get_ip_address() : "None"), (sockAddr.get_port() ? sockAddr.get_port() : 0)); //check setting

    int rc = mqttNetwork.connect(sockAddr); //(host, 1883);
    if (rc != 0)
    {
        printf("Connection error.");
        return -1;
    }
    printf("Successfully connected!\r\n");

    MQTTPacket_connectData data = MQTTPacket_connectData_initializer;
    data.MQTTVersion = 3;
    data.clientID.cstring = "Mbed";

    if ((rc = client.connect(data)) != 0)
    {
        printf("Fail to connect MQTT\r\n");
    }
    if (client.subscribe(topic, MQTT::QOS0, messageArrived) != 0)
    {
        printf("Fail to subscribe\r\n");
    }

    int num = 0;
    while (num != 5)
    {
        client.yield(100);
        ++num;
    }
    /*-------------Start thread------------*/

    mqtt_thread.start(callback(&queue, &EventQueue::dispatch_forever));
    tilt_thread.start(callback(&tilt_angle, &client));
    UI_thread.start(&angle_sel);
    btn2.rise(queue.event(&publish_sel_angle, &client));
    //btn3.rise(&close_mqtt);

    /*-------------WIFI------------*/
    while (1)
    {
        if (closed) break;
        client.yield(500);
        ThisThread::sleep_for(500ms);
    }
    printf("Ready to close MQTT Network......\n");

    if ((rc = client.unsubscribe(topic)) != 0)
    {
        printf("Failed: rc from unsubscribe was %d\n", rc);
    }
    if ((rc = client.disconnect()) != 0)
    {
        printf("Failed: rc from disconnect was %d\n", rc);
    }

    mqttNetwork.disconnect();
    printf("Successfully closed!\n");
}

int main() {

    BSP_ACCELERO_Init();

    //The mbed RPC classes are now wrapped to create an RPC enabled version - see RpcClasses.h so don't add to base class

    // receive commands, and send back the responses
    char buf[256], outbuf[256];

    FILE *devin = fdopen(&pc, "r");
    FILE *devout = fdopen(&pc, "w");

    wifi_thread.start(&wifi_set);

    while (1) {
        memset(buf, 0, 256);
        for (int i = 0; i < 255; i++) {
            char recv = fgetc(devin);
            if (recv == '\r' || recv == '\n') {
                printf("\r\n");
                break;
            }
            buf[i] = fputc(recv, devout);
        }
        //Call the static call method on the RPC class
        RPC::call(buf, outbuf);
        printf("%s\r\n", outbuf);
    }

    return 0;
}

