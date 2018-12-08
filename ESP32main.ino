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

//    const char* ssid     =    "iouti_net";
//    const char* password =    "thenightmareofhackers";
//    const char* raspip =      "192.168.5.1";
    const char* ssid     =    "Aquaris X5 Plus";
    const char* password =    "3cdb401cb5d6";
//    const char* raspip =      "192.168.43.81";
//    const char* ssid     =    "AndroidAP";
//    const char* password =    "wqzz8899";
//    const char* raspip =      "192.168.1.143";
//    const char* ssid     =    "Celia";
//    const char* password =    "ww25i16axkg2e";
    const char* raspip =      "192.168.43.14";
    String esp_id="";

// Synch beacon timint
    const unsigned long lastBeacon;

    const int port = 8080;


//INPUT DEFINITIONS
    #define CHANNEL 36
    #define SCL_INDEX 0x00
    #define SCL_TIME 0x01
    #define SCL_FREQUENCY 0x02
    #define SCL_PLOT 0x03
    const uint16_t samples = 1024; //This value MUST ALWAYS be a power of 2
    const int Nwave = 1024;
    const double samplingFrequency = 16000; //Hz

    //INPUT VARIABLES
    unsigned int sampling_period_us;
    unsigned long microsecondsFFT;
    unsigned long microsecondsLectura;
    boolean isVoice = 0;
    int noVoiceCounter = 0;
    int tramas4Segundos = 32;
    volatile boolean calcularFFT = 1;
    int numTramaFFT = 0;
    int numTramaLectura = 0;
    volatile boolean tramaNueva = 0;
    volatile boolean concat;
    double volumen;
    volatile int numTramasGuardadas = 0;
    const int tramas1Segundo = 16;
    volatile boolean silencio = 0;
    boolean quieroVolumen;
    int tramas10Segundos = 156;
    int numTramasEnviadas;

    /*
    These are the input and output vectors
    Input vectors receive computed results from FFT
    */
    double vReal[samples];
    double vImag[samples];
    double tramaFFT[Nwave];
    double wave[Nwave];
    double waveForFFT[Nwave];
    String waveAntS;
    String waveAntS2;
    String savedAudio;
    String waveString;


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
    int localitzationDelta=45;
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
      bits_per_sample: I2S_BITS_PER_SAMPLE_32BIT, //(i2s_bits_per_sample_t)48,    // Only 8-bit DAC support
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
    numTramaLectura++;
    
    
    int32_t mic_sample = 0;
    
    for(int i=0; i<Nwave; i++) //Bucle que lee muestras de audio
    { 
  
      //read 24 bits of signed data into a 48 bit signed container
      if (i2s_pop_sample(I2S_NUM_1, (char*)&mic_sample, portMAX_DELAY) == 4) {
    
        //Porting a 23 signed number into a 32 bits we note sign which is '-' if bit 23 is '1'
        mic_sample = (mic_sample & 0x400000) ?
                     //Negative: B/c of 2compliment unused bits left of the sign bit need to be '1's
                     (mic_sample | 0xFF800000) :
                     //Positive: B/c of 2compliment unused bits left of the sign bit need to be '0's
                     (mic_sample & 0x7FFFFF);
    
           //mic_sample <<= 1; //scale back up to proper 3 byte size unless you don't care
    
        //printBarForGraph(abs(mic_sample));
        wave[i] = mic_sample;
        waveString += ((String)wave[i] + ","); //AMB COMA??
        
      }
  
    }
  
    
      
      for(int i=0; i<Nwave; i++){
  
            waveForFFT[i] = wave[i];
      }
       
  
   
  
    
       if (!concat){
       //Guardar tramas anteriores
        waveAntS2 = waveAntS;
        waveAntS = waveString;
        savedAudio = waveAntS2 + waveAntS + waveString;
        numTramasGuardadas = 3;
  
       }else{
       //Concantenar el audio que vas recibiendo
        savedAudio += waveString;
        Serial.println("Concatenando...");
        numTramasGuardadas++;
        
       }

       if(numTramasGuardadas >= 300){
        concat = 0;
       }
  
   
  
    calcularFFT = !calcularFFT;
    tramaNueva = 1;
    
    vTaskDelay(2);
  
     if(calcularFFT == 1){
      vTaskResume(taskFFT);
      //Serial.println("arribaResume");
    }
  
      //Serial.println("arribaFinal");
  }

}
        
        

void computeFFT(void *parameter){

   int localState;
   int localRaspiListening;
//   boolean localTramaNueva;
//   boolean localCalcularFFT;

   Serial.println("------------------->> Entra FFT");

  

  while(true){

    numTramaFFT = numTramaFFT + 2;
    microsecondsFFT = micros();

   for(int i=0; i<samples; i++) //Bucle que lee 1024 muestras de audio
    {
      vReal[i] = wave[i];
      //Serial.println("-----------------> vReal[" +(String)i + "] (" + (String)numTramaFFT + ") = " + (String)vReal[i]);
      tramaFFT[i] = wave[i];
      vImag[i] = 0;

    }

    arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */  //Freq + alta 4482.421875Hz
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);

    if(x > 85.0 && x < 255.0){
        Serial.println("-----------------------Voz");
        isVoice = 1;
        noVoiceCounter = 0;
        volumen = computeVolume(tramaFFT);
        Serial.println(x);

    }else{
      Serial.println("Otra cosa");
      Serial.println(x);
      isVoice = 0;
      noVoiceCounter++;
    }

    
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

    if(localState == IDLE && localRaspiListening == 0){
      
         noVoiceCounter = 0;
        savedAudio= " ";
        concat = 0;
        numTramasGuardadas = 0;
        
    }else if(localState == IDLE && isVoice == 1 && localRaspiListening == 1){
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
      vTaskResume(esp32Client);

     }


    //Para de enviar cuando pasan 4 segundos de silencio o maximo 10 segundos de

    if(localState == AUDIO){
      Serial.print("Num Trama: ");
      Serial.println(numTramasGuardadas);
      
 /*     if( numTramasGuardadas == tramas1Segundo){ //Envia audio de 1 segundo cada vez

        //envia audio

        
        savedAudio = " ";
        //asignar con semaforo
        vTaskResume(esp32Client);
        
        numTramasEnviadas += numTramasGuardadas;
        numTramasGuardadas = 0;
        Serial.println("enviando...");
      }else */
      
      if((numTramasGuardadas >= tramas10Segundos || noVoiceCounter >= tramas4Segundos)){
        
        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            data = savedAudio;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }

        Serial.println(savedAudio);

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
        
        noVoiceCounter = 0;
        savedAudio= " ";
        concat = 0;
        numTramasGuardadas = 0;
      }

    }


   // tramaNueva = 0;
   //Serial.println("S'adorm");
    vTaskSuspend(taskFFT);
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
                        digitalWrite(33, HIGH);               // GET /H turns the REALY on
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("H request detected");
                        client.stop();

                    }else if (currentLine.endsWith("GET /L ")) {
                        digitalWrite(33, LOW);                // GET /L turns the RELAY off
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
                    }else if (currentLine.endsWith("GET /sync ")) {
                        // VOLUME REQUEST FROM RASPI
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("Sync in action");
                        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            setLastBeacon(micros());
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( dataSemaphore );
                        }
                        client.stop();
                        Serial.print("SYNC");
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
                        client.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile,esp_id));
                        Serial.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile,esp_id));
                                             
                        state_env=WAITRESPONSE;
                        
                    break;
                    case AUDIO:
                        client.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile,esp_id));
                        Serial.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile,esp_id));

                        
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
                        client.println(makeHTTPrequest("POST","/error","application/json",data , localitzationDelta, EndOfFile,esp_id));
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
    pinMode(33, OUTPUT);       // set the RELAY pin mode
    pinMode(32, OUTPUT);      // set the RELAY pin mode
    Serial.begin(115200);
    sampling_period_us = round(1000000*(1.0/samplingFrequency));

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
    esp_id=WiFi.localIP().toString();
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(33, LOW);
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
        computeFFT,             //Task function
        "FFT",                 //name of task
        1000,                     //Stack size of the task
        NULL,                     //parameter of the task
        1,                        //priority of the task
        &taskFFT,                   //Task handle to keep track of created task
        0);                       //core


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
        dataToSend = data;
        localitzationToSend = String(localitzationDelta);
        EOFtoSend=EndOfFile;
        // We have finished accessing the shared resource.  Release the
        // semaphore.
        xSemaphoreGive( dataSemaphore );
    }
    if(uri=="/audio"){
        postBody=postBody+
        "{\n"
        " \"esp_id\": \""+esp_id+"\",\n"
        " \"EOF\": \""+EOFtoSend+"\",\n"
        " \"location\": \""+ localitzationToSend +"\",\n"
        " \"data\": \""+dataToSend+"\"\n"
        "}\n";
//        postBody=postBody+
//        "{\n"+
//        " \"esp_id\": \""+esp_id+"\",\n"+
//        " \"timestamp\": \""+ localitzationToSend +"\",\n"+
//        " \"delay\": \""+ localitzationToSend +"\",\n"+
//        " \"volume\": \""+dataToSend+"\"\n"+
//        "}\n";
    }else{
        postBody=postBody+
        "{\n"+
        " \"esp_id\": \""+esp_id+"\",\n"+
        " \"timestamp\": \""+ localitzationToSend +"\",\n"+
        " \"delay\": \""+ localitzationToSend +"\",\n"+
        " \"volume\": \""+dataToSend+"\"\n"+
        "}\n";
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

void setLastBeacon(unsigned long time){
    lastBeacon = time;
}

unsigned long getLastBeacon(){
    return lastBeacon;   
}

unsigned long getDelay(unsigned long currTime){
    unsigned long delay = currTime-getLastBeacon();
    return delay;
}
