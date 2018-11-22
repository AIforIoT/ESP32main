//NEW ESP32 code

#include <WiFi.h>

//WIFI DEFINITIONS
int status = WL_IDLE_STATUS;
const char* ssid     = "iouti_net";
const char* password = "thenightmareofhackers";
const char* raspip = "192.168.5.1";
const int port = 8080;


// Initialize the Wifi client library
WiFiClient client;
// Initialize the Wifi server library
WiFiServer server(80);

//GLOBAL STATES DEFINITIONS
const int NODATA    = 0;
const int VOLUME    = 1;
const int DATA      = 2;
volatile int state=NODATA; //INITIAL STATE
//GLOBAL DATA VARIABLES
SemaphoreHandle_t dataSemaphore = NULL;
String data = "Hola TEST";
int localitzationDelta=100;
boolean EndOfFile;

//TASK HANDLES DEFINITIONS
TaskHandle_t Beacons;
TaskHandle_t MicroInput;
TaskHandle_t esp32Server;
TaskHandle_t esp32Client;

void codeForBeacons( void * parameter){
    //Code goes here
    Serial.begin(115200);
    while(true){
        //Serial.println("Print from core 1,Beacon task test");
        vTaskDelay(500);
    }
}

void codeForMicroInput( void * parameter){
    //Code goes here
    Serial.begin(115200);
    while(true){
        //Serial.println("Print from core 0,MicroInput task test");
        vTaskDelay(50);
    }
}

void codeForServer( void * parameter){
    pinMode(5, OUTPUT);      // set the RELAY pin mode
    pinMode(18, OUTPUT);      // set the RELAY pin mode
    pinMode(19, OUTPUT);      // set the RELAY pin mode
    pinMode(21, OUTPUT);      // set the RELAY pin mode
    server.begin();
    delay(2000);
    Serial.println("Server started");
    Serial.print("Server is at ");
    Serial.println(WiFi.localIP());
    delay(2000);
    char c;
    //Loop
    while(true){
        // listen for incoming clients
        WiFiClient client = server.available();
        if (client) {
            Serial.println("new client");
            String currentLine = "";                // make a String to hold incoming data from the client
            while (client.connected()) {            // loop while the client's connected
                if (client.available()) {             // if there's bytes to read from the client,
                  c = client.read();             // read a byte, then
                  Serial.write(c);                    // print it out the serial monitor
                }
                //This ESP32 server only expects HTTP Request:
                // GET /H HTTP/1.x
                // GET /L HTTP/1.x
                if (c != '\r') {  // if you got anything else but a carriage return character,
                    currentLine += c;      // add it to the end of the currentLine
                    // Check to see if the client request was "GET /H" or "GET /L":
                    if (currentLine.endsWith("GET /H ")) {
                        digitalWrite(5, HIGH);               // GET /H turns the REALY on
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("H request detected");
                        client.stop();

                    }else if (currentLine.endsWith("GET /L ")) {
                        digitalWrite(5, LOW);                // GET /L turns the RELAY off
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("L request detected");
                        client.stop();


                    }else if (currentLine.endsWith("GET /data/on ")) {
                        // DATA REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("DATA request detected");

                        //DATA CODE GOES HERE...
                        state=DATA;
                        digitalWrite(18, HIGH);
                        vTaskDelay(2000);
                        digitalWrite(18, LOW);
                        client.stop();

                    }else if (currentLine.endsWith("GET /data/off ")) {
                        // DATA REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("NODATA request detected");
                        //NODATA CODE GOES HERE...
                        state=NODATA;
                        digitalWrite(18, HIGH);
                        vTaskDelay(2000);
                        digitalWrite(18, LOW);
                        client.stop();

                    }else if (currentLine.endsWith("GET /volume ")) {
                        // DATA REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("VOLUME request detected");
                        state=VOLUME;
                        //VOLUME CODE GOES HERE...
                        digitalWrite(19, HIGH);
                        vTaskDelay(2000);
                        digitalWrite(19, LOW);
                        client.stop();
                    }

                }else{  //End of first request line , no accepted get requested
                    //Error 405 Method Not Allowed
                    client.println("HTTP/1.1 405 Method Not Allowed");
                    client.println();
                    digitalWrite(21, HIGH);
                    vTaskDelay(2000);
                    digitalWrite(21, LOW);
                    client.stop();
                }
            }
            // close the connection:

            Serial.println("Client Disconnected.");
        }
    }//END WHILE
}//END code for server

void codeForClient( void * parameter){
    //Code goes here
    while(true){
        vTaskSuspend(esp32Client);
        client.stop();
        vTaskDelay(1000);
        //if there's a successful connection:
        if (client.connect(raspip, port)) {
            Serial.println("connecting...");
            // send the HTTP request:
            vTaskDelay(100);
            //Send information
            switch (state) {
                case VOLUME:
                    client.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile));
                    break;
                case DATA:
                    client.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile));
                    break;
                default:
                    client.println(makeHTTPrequest("POST","/error","application/json",data , localitzationDelta, EndOfFile));
                    break;
            }
            Serial.println("Post end");
        }else{
            // if you couldn't make a connection:
            Serial.println("connection failed");
            //FATAL ERROR
        }
    }
}//END code for client

void setup(){

    Serial.begin(115200);

    vSemaphoreCreateBinary( dataSemaphore );

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, password);
        delay(2000);
    }
    Serial.println("Connected");

    xTaskCreatePinnedToCore(
        codeForBeacons,             //Task function
        "Beacons",                  //name of task
        1000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &Beacons,                   //Task handle to keep track of created task
        1);                         //core

    xTaskCreatePinnedToCore(
        codeForMicroInput,          //Task function
        "MicroInput",               //name of task
        5000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &MicroInput,                //Task handle to keep track of created task
        0);                         //core

    xTaskCreatePinnedToCore(
        codeForServer,              //Task function
        "Server",                   //name of task
        5000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &esp32Server,               //Task handle to keep track of created task
        1);                         //core

    xTaskCreatePinnedToCore(
        codeForClient,              //Task function
        "Client",                   //name of task
        5000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &esp32Client,               //Task handle to keep track of created task
        1);                         //core

    state=DATA;
    vTaskResume( esp32Client );
    state=VOLUME;
    vTaskResume( esp32Client );
    state=50;
    vTaskResume( esp32Client );

}

void loop(){
    // send it out the serial port.This is for debugging purposes only:
//    while (client.available()) {
//            char c = client.read();
            //Serial.write(c);
//    }
    while(true){
        //Serial.println(state);
    }
    vTaskDelay(5000);
}

//Auxiliary functions
String makeHTTPrequest(String method, String uri, String type, String data, int localitzationDelta, boolean EndOfFile){
    Serial.print("POST REQUESTSEND");
    String dataToSend = "";
    String localitzationToSend="";
    String postBody="";
    boolean EOFtoSend;
    if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
    {
        // We were able to obtain the semaphore and can now access the
        // shared resource.
        //We make a local copy
        dataToSend = data;
        localitzationToSend = String(localitzationDelta);
        EOFtoSend=EndOfFile;
        // We have finished accessing the shared resource.  Release the
        // semaphore.
        xSemaphoreGive( dataSemaphore );
    }
    postBody=postBody+
    "{\n"
    " \"EOF\": \""+EOFtoSend+"\",\n"
    " \"location\": \""+ localitzationToSend +"\",\n"
    " \"data\": \""+dataToSend+"\"\n"
    "}\n";

    String postHeader=
    method+" "+uri+" HTTP/1.0\n"
    "content-type: "+type+"\n"
    "content-Length: "+postBody.length()+"\n\n";

    return postHeader+postBody;
}
