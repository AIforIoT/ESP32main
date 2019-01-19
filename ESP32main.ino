//NEW ESP32 code

#include <WiFi.h>
#include "arduinoFFT.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s.h"
#include "esp_system.h"
#include <stdio.h>
#include <stdint.h>

//WIFI DEFINITIONS

    int status = WL_IDLE_STATUS;

    const char* ssid     =    "iouti_net";
    const char* password =    "thenightmareofhackers";
//    const char* raspip =      "192.168.5.1";
//    const char* ssid     =    "Aquaris X5 Plus";
//    const char* password =    "3cdb401cb5d6";
//    const char* raspip =      "192.168.43.81";
//    const char* ssid     =    "AndroidAP";
//    const char* password =    "wqzz8899";
//    const char* raspip =      "192.168.1.143";
//    const char* ssid     =    "Celia";
//    const char* password =    "ww25i16axkg2e";
    const char* raspip =      "192.168.5.1";
    String esp_ip="";

    const int port = 8080;


//INPUT DEFINITIONS
    #define CHANNEL 36
    #define SCL_INDEX 0x00
    #define SCL_TIME 0x01
    #define SCL_FREQUENCY 0x02
    #define SCL_PLOT 0x03
    const uint16_t Nwave = 1024; //This value MUST ALWAYS be a power of 2
//    const int Nwave = 1024;
    const double samplingFrequency = 16000; //Hz

    //INPUT VARIABLES
    boolean isVoice = 0;
    int tramas4Segundos = 32;
    volatile boolean calcularFFT = 1;
    volatile boolean tramaNueva = 0;
    volatile boolean concat = 0;
    volatile boolean newAudio = 1;
    double volumen;
    volatile int numTramasGuardadas = 0;
    const int tramas1Segundo = 16;
    volatile boolean silencio = 0;
    boolean quieroVolumen;
    int tramas10Segundos = 22; //156;
    int numTramasEnviadas;
    int numTramasTotal;
    int bufferPrevSize = 10; //En tramas 

    char byte1 = 0;
    char byte2 = 0;

    /*
    These are the input and output vectors
    Input vectors receive computed results from FFT
    */
    double vReal[Nwave];
    double vImag[Nwave];
    double tramaFFT[Nwave];
    double wave[Nwave];
    double waveForFFT[Nwave];
    String waveAntS;
    String waveAntS2;
    String savedAudio;
    String waveString;
    String audioToSend = "";
      
    //String audioCopiat = "-4\n-4\n-5\n-5\n-4\n-4\n-3\n-3\n-4\n-4\n-3\n-3\n-4\n-4\n-2\n-2\n-2\n-2\n0\n0\n1\n1\n0\n";

    int cont = 0;


//WIFI INIT

    // Initialize the Wifi client library
    WiFiClient client;
    // Initialize the Wifi server library
    WiFiServer server(80);


//GLOBAL STATES DEFINITIONS

    const int IDLE              = 0;
    const int VOLUME            = 1;
    const int WAITRESPONSE      = 2;
    const int AUDIO             = 3;

    //These must be readed using stateSemaphore
    volatile int state_env      = IDLE;
    volatile boolean raspiListening=    true;


//GLOBAL STATE VARIABLES

    SemaphoreHandle_t dataSemaphore = NULL;
    SemaphoreHandle_t stateSemaphore = NULL;
    SemaphoreHandle_t fftSemaphore = NULL;
    String data = "";
    int localitzationDelta=random(30, 100);
    boolean EndOfFile;


//TASK HANDLES DEFINITIONS
    TaskHandle_t Beacons;
    TaskHandle_t MicroInput;
    TaskHandle_t taskFFT;
    TaskHandle_t Enviar;
    TaskHandle_t esp32Server;
    TaskHandle_t esp32Client;

//MICRO CONFIG
    #define AUDIO_RATE 16000

namespace {

const int sample_rate = 16000;

/* RX: I2S_NUM_1 */
i2s_config_t i2s_config_rx  = {
mode: (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
sample_rate: sample_rate,
bits_per_sample: I2S_BITS_PER_SAMPLE_16BIT, //(i2s_bits_per_sample_t)48,    // Only 8-bit DAC support
channel_format: I2S_CHANNEL_FMT_ONLY_RIGHT,   // 2-channels
communication_format: (i2s_comm_format_t)(I2S_COMM_FORMAT_I2S | I2S_COMM_FORMAT_I2S_LSB),
intr_alloc_flags: ESP_INTR_FLAG_LEVEL1,        // Interrupt level 1
  dma_buf_count: 8,                            // number of buffers, 128 max.
  dma_buf_len: 8                          // size of each buffer
};

i2s_pin_config_t pin_config_rx = {
bck_io_num: GPIO_NUM_26,
ws_io_num: GPIO_NUM_25,
data_out_num: I2S_PIN_NO_CHANGE,
data_in_num: GPIO_NUM_22
};
};

//Auxiliary functions
String makeHTTPrequest(String method, String uri, String type, String data, int localitzationDelta, boolean EndOfFile, String esp_id){
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
        //dataToSend = data;
        localitzationToSend = String(localitzationDelta);
        EOFtoSend=EndOfFile;
        // We have finished accessing the shared resource.  Release the
        // semaphore.
       
            postBody=postBody+
            "{\n"+
            " \"esp_id\": \""+esp_id+"\",\n"+
            " \"timestamp\": \""+ localitzationToSend +"\",\n"+
            " \"delay\": \""+ localitzationToSend +"\",\n"+
            " \"volume\": \""+data+"\"\n"+
            "}\n";
        
        xSemaphoreGive( dataSemaphore );
    }

    String postHeader=
    method+" "+uri+" HTTP/1.1\n"
    "content-type: "+type+"\n"
    "content-Length: "+postBody.length()+"\n\n";

    return postHeader+postBody;
}
double computeVolume(double *wave){
    double sumP = 0;
    double pot = 0;
    for(int i=0; i<Nwave; i++){
        sumP = sumP + (wave[i]*wave[i]);
    }
    pot = sumP/Nwave;
    sumP = 0;

    return pot;

}
String makeHTTPaudio(String esp_ip, boolean EndOfFile, String postBody){

    String header="POST /audio/"+esp_ip+"/"+String(EndOfFile)+" HTTP/1.1\n";
    header=header+"content-type: text/plain\n"+"content-Length: "+String(postBody.length())+"\n\n";
    return header+postBody;
}



void codeForBeacons( void * parameter){
    //Code goes here
    Serial.begin(115200);
    while(true){
        //Serial.println("Print from core 1,Beacon task test");
        vTaskDelay(500);
    }
}

void codeForMicroInput( void * parameter){
    /*SAMPLING*/
  while(true){

    int16_t mic_sample = 0;
    int16_t mic_val = 0;

    for(int i=0; i<Nwave; i++) //Bucle que lee muestras de audio
    {

      //read 24 bits of signed data into a 48 bit signed container
     if (i2s_pop_sample(I2S_NUM_1, (char*)&mic_sample, portMAX_DELAY) == 2) {

      //like: https://forums.adafruit.com/viewtopic.php?f=19&t=125101 ICS-43434 is 1 pulse late
      //The MSBit is actually a false bit (always 1), the MSB of the real data is actually the second bit

      //Porting a 23 signed number into a 32 bits we note sign which is '-' if bit 23 is '1'
      mic_val = (mic_sample & 0x4000) ?
                   //Negative: B/c of 2compliment unused bits left of the sign bit need to be '1's
                   (mic_sample | 0x8000) :
                   //Positive: B/c of 2compliment unused bits left of the sign bit need to be '0's
                   (mic_sample & 0x7FFF);

           //mic_sample <<= 1; //scale back up to proper 3 byte size unless you don't care
       
        wave[i] = mic_val;
        /*per enviar nums creixents*/
        //cont++;
        //mic_sample = cont%255;
        /**/
        /*CODIFICATION*/
        byte1 = mic_sample&(0x00FF);  //El byte mes significatiu
        byte2 = mic_sample >> 8;      //El byte menys significatiu
        /*CODIFICATION ZEROS*/ //NULL is not sent, so we can't send '0'
        if ((mic_sample >> 8) == 0 && (mic_sample&(0x00FF)) == 0){ //In case both byte are 0
          waveString += ((String)(char)10000000 + (String)(char)00000001);
        }else if((mic_sample&(0x00FF)) == 0){ //In case byte1 is 0
          waveString += ((String)byte2 + (String)(char)00000001);
        }else if((mic_sample >> 8) == 0){ //In case byte2 is 0
          waveString += ((String)(char)10000000 + (String)byte1);
        }else{ 
          waveString += ((String)byte2 + (String)byte1);
        }

      }

    }

      
       if (!concat && newAudio){ //To save audio previous to detecting voice  
            if(savedAudio.length() >= 2*bufferPrevSize*Nwave){
              savedAudio.remove(0, 2*Nwave);
              savedAudio += waveString;
              waveString = "";          
              numTramasGuardadas = bufferPrevSize;                    
            } else {
              savedAudio += waveString;
              waveString = "";
              numTramasGuardadas++;
              Serial.println("---------------------------------------ESPERA...");
            }
       }else if(!concat && !newAudio){ //To not concatenate or save previous audio      
       //BUIT
       }else if(concat){  //To concatenate certain seconds of audio while sending it
       //Concantenar el audio que vas recibiendo
        savedAudio += waveString;
        waveString = "";
        //Serial.println("Concatenando...");
        numTramasGuardadas++;
        numTramasTotal++;
       }
    
  }
}



void computeFFT(void *parameter){

   /*Local variables*/
   int localState;
   int localRaspiListening;

  while(true){

   for(int i=0; i<Nwave; i++) //Reading of 1024 samples of audio
   {
      vReal[i] = wave[i];
      tramaFFT[i] = wave[i];
      vImag[i] = 0;

    }

    arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

    FFT.Windowing(vReal, Nwave, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, Nwave, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, Nwave); /* Compute magnitudes */
    double x = FFT.MajorPeak(vReal, Nwave, samplingFrequency);

    if(x > 85.0 && x < 255.0){ //x is the peak frequency, it detects voice if this value is between 85 Hz and 255 Hz
        //Serial.println("-----------------------Voz");
        isVoice = 1;
        volumen = computeVolume(tramaFFT);
    }else{
      isVoice = 0;
    }

    localitzationDelta=random(30, 10000); //BORRARRRRRRRRRRRRRRRRR cuando este lo otro

    //The value of state_env and raspiListening depend on the Server thread
    if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
    {
        // We were able to obtain the semaphore and can now access the
        // shared resource.
        localState = state_env;
        localRaspiListening = raspiListening;
        // We have finished accessing the shared resource.  Release the
        // semaphore.
        xSemaphoreGive( stateSemaphore );
    }

    /*TO SEND*/

    if(localState == IDLE && localRaspiListening == 0){
                    
        concat = 0;
        numTramasGuardadas = 0;

    }else if(localState == IDLE && isVoice == 1 && localRaspiListening == 1 && savedAudio.length() >= 2*bufferPrevSize*Nwave){ 
          concat = 1;

      if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
      {
          // We were able to obtain the semaphore and can now access the
          // shared resource.
          state_env = VOLUME;
          // We have finished accessing the shared resource.  Release the
          // semaphore.
          xSemaphoreGive( stateSemaphore );
      }

        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            data = (String)volumen;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }

      //asignar con semaforo
      Serial.println("-------------------------------------------->Petición");
      Serial.println((String)volumen);
      //Serial.println(savedAudio.length());
      vTaskResume(esp32Client);

     }

    if(localState == AUDIO){

      if(savedAudio.length() <= 2*3*Nwave){ //When there is less than 2*3*1024 characters in savedAudio, it is sent with the End of File

        //Serial.println(prova);

        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            data = savedAudio;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }

        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            EndOfFile = true;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }

        vTaskResume(esp32Client);
        Serial.println("Enviado");

        /*Reiniciamos variables*/
        savedAudio= "";
        newAudio = 1;
        numTramasEnviadas = 0;
        numTramasGuardadas = 0;
        numTramasTotal = 0;

      } else { 

       int indFin = Nwave*2*3; //3 tramas
       if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
         {
             // We were able to obtain the semaphore and can now access the
             // shared resource.
             //data = savedAudio;
             data = savedAudio.substring(0, indFin);
             // We have finished accessing the shared resource.  Release the
             // semaphore.
             xSemaphoreGive( dataSemaphore );  
         }
         
         savedAudio.remove(0, indFin); 

         vTaskResume(esp32Client);
         
         /*Provisional*/
        /* Serial.print("Tramas Enviadas: ");
         Serial.println(numTramasEnviadas);
         Serial.print("Tramas Guardadas: ");
         Serial.print(numTramasGuardadas);
         Serial.print("-->");
         Serial.println(savedAudio.length());
         Serial.print("-->");
         Serial.println(data.length());
         Serial.print("Tramas Total: ");
         Serial.println(numTramasTotal);*/
         /**/

         numTramasEnviadas += 6;
         numTramasGuardadas -= 6;

         //Serial.println("enviando...");
         if((numTramasTotal >= tramas10Segundos)){
           newAudio = 0;
           concat = 0;
           waveAntS = "";
           waveAntS2 = "";              
        }
    }

   // vTaskSuspend(taskFFT);
   vTaskDelay(300);
  }
 }
}


void codeForServer( void * parameter){

    server.begin();
    vTaskDelay(2000);
    Serial.println("Server started");
    Serial.print("Server is at ");
    Serial.println(WiFi.localIP());
    vTaskDelay(2000);
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
                        digitalWrite(19, HIGH);               // GET /H turns the REALY on
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("H request detected");
                        client.stop();

                    }else if (currentLine.endsWith("GET /L ")) {
                        digitalWrite(19, LOW);                // GET /L turns the RELAY off
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("L request detected");
                        client.stop();


                    }else if (currentLine.endsWith("GET /data/on ")) {
                        // AUDIO DATA REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            state_env=AUDIO;
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( stateSemaphore );
                        }
                        Serial.print("DATAON");
                        client.stop();

                     }else if (currentLine.endsWith("GET /data/off ")) {
                        // AUDIO DATA NOT REQUESTED FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            state_env=IDLE;
                            raspiListening=false;
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( stateSemaphore );
                        }
                        Serial.print("DATAOFF");
                        client.stop();

                    }else if (currentLine.endsWith("GET /volume ")) {
                        // VOLUME REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("VOLUME request detected");
                        if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            raspiListening=true;
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( stateSemaphore );
                        }
                        client.stop();
                        Serial.print("VOLUME");
                    }

                }else{  //End of first request line , no accepted get requested
                    //Error 405 Method Not Allowed
                    client.println("HTTP/1.1 405 Method Not Allowed");
                    client.println();
                    client.stop();
                }
            }
            // close the connection:
            Serial.println("Client Disconnected.");
            vTaskDelay(50);
        }
    }//END WHILE
}//END code for server

void codeForClient( void * parameter){
    //Code goes here
    while(true){
        vTaskSuspend(esp32Client);
        client.stop();
        //if there's a successful connection:
        if (client.connect(raspip, port)) {
            Serial.println("connecting...");
            // send the HTTP request:
            //Send information
            Serial.println("Information to send");
            if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.
                switch (state_env) {
                    case VOLUME:
                        client.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile,esp_ip));
                        Serial.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile,esp_ip));

                        state_env=WAITRESPONSE;

                    break;
                    case AUDIO:
                        //client.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile,esp_id));
                        //Serial.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile,esp_id));
                        client.println(makeHTTPaudio(esp_ip,EndOfFile,data));
                        Serial.println(makeHTTPaudio(esp_ip,EndOfFile,data));

                        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            if(EndOfFile==true){
                                EndOfFile=false;
                                //Go back to initial state
                                state_env=IDLE;
                                raspiListening=false;
                            }
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( dataSemaphore );
                        }
                    break;
                    default:
                        client.println(makeHTTPrequest("POST","/error","application/json",data , localitzationDelta, EndOfFile,esp_ip));
                    break;
                    }
                Serial.println("Post end");
                // We have finished accessing the shared resource.  Release the
                // semaphore.
                xSemaphoreGive( stateSemaphore );
            }

        }else{
            // if you couldn't make a connection:
            Serial.println("connection failed");
            if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
            {
                // We were able to obtain the semaphore and can now access the
                // shared resource.
                state_env=IDLE;
                raspiListening=false;
                // We have finished accessing the shared resource.  Release the
                // semaphore.
                xSemaphoreGive( stateSemaphore );
            }
            //FATAL ERROR
        }
    }
}//END code for client


//Start execution
void setup(){
    pinMode(19, OUTPUT);       // set the RELAY pin mode
    pinMode(32, OUTPUT);      // set the RELAY pin mode
    Serial.begin(115200);

    vSemaphoreCreateBinary( dataSemaphore );
    vSemaphoreCreateBinary( stateSemaphore );
    vSemaphoreCreateBinary( fftSemaphore );

    i2s_driver_install(I2S_NUM_1, &i2s_config_rx, 0, NULL);
    i2s_set_pin(I2S_NUM_1, &pin_config_rx);
    i2s_zero_dma_buffer(I2S_NUM_1);

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, password);
        delay(2000);
    }
    Serial.println("Connected");
    esp_ip=WiFi.localIP().toString();
    digitalWrite(19, HIGH);
    delay(1000);
    digitalWrite(19, LOW);
    delay(1000);
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);
//    xTaskCreatePinnedToCore(
//        codeForBeacons,             //Task function
//        "Beacons",                  //name of task
//        1000,                       //Stack size of the task
//        NULL,                       //parameter of the task
//        1,                          //priority of the task
//        &Beacons,                   //Task handle to keep track of created task
//        1);                         //core




    xTaskCreatePinnedToCore(
        codeForMicroInput,          //Task function
        "MicroInput",               //name of task
        5000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &MicroInput,                //Task handle to keep track of created task
        0);                         //core


   xTaskCreatePinnedToCore(
        computeFFT,                 //Task function
        "FFT",                      //name of task
        1000,                       //Stack size of the task
        NULL,                       //parameter of the task
        1,                          //priority of the task
        &taskFFT,                   //Task handle to keep track of created task
        1);                         //core
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


    //vTaskSuspend(Enviar);


}
//DO NOTHING IN LOOP
void loop(){
    // send it out the serial port.This is for debugging purposes only:
//    while (client.available()) {
//            char c = client.read();
            //Serial.write(c);
//    }
//    while(true){
        //Serial.println(state_env);
//    }
    vTaskDelay(1000);
    Serial.println("----------------------------------->" + (String)state_env);
}
