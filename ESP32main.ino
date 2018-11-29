//NEW ESP32 code

#include <WiFi.h>
#include "arduinoFFT.h"
#include "string.h"

//WIFI DEFINITIONS

    int status = WL_IDLE_STATUS;
//    const char* ssid     =    "iouti_net";
//    const char* password =    "thenightmareofhackers";
//    const char* raspip =      "192.168.5.1";
    const char* ssid     =    "Aquaris X5 Plus";
    const char* password =    "3cdb401cb5d6";
    const char* raspip =      "192.168.43.81";
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
    String data = "";
    int localitzationDelta=100;
    boolean EndOfFile;


//TASK HANDLES DEFINITIONS
    TaskHandle_t Beacons;
    TaskHandle_t MicroInput;
    TaskHandle_t FFT;
    TaskHandle_t Enviar;
    TaskHandle_t esp32Server;
    TaskHandle_t esp32Client;


void codeForBeacons( void * parameter){
    //Code goes here
    Serial.begin(115200);
    while(true){
        Serial.println("Print from core 1,Beacon task test");
        delay(100);
        vTaskDelay(500/portTICK_PERIOD_MS);
    }
}

void codeForMicroInput( void * parameter){
    //Code goes here
  while(true){

    /*SAMPLING*/
      microsecondsLectura = micros();
      numTramaLectura++;


     for(int i=0; i<Nwave; i++) //Bucle que lee Nwave muestras de audio
     {
        tramaNueva = 0;
        //Serial.println("Proceso 2");
        wave[i] = analogRead(CHANNEL); // Lectura del valor del pin
        waveString += (String)wave[i]; //AMB COMA??

        long t0 = micros();
        while(micros() - microsecondsLectura < sampling_period_us){ //Espera a que haya pasado un periodo de muestreo
           vTaskDelay(50);
        }
        long t1 = micros();
        vTaskDelay(50);
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
      //Serial.println("Concatenando...");
      numTramasGuardadas++;
     }

    calcularFFT = !calcularFFT;
    tramaNueva = 1;

    long t2 = micros() - microsecondsLectura;
    long fm = 1000000*Nwave/t2;
    vTaskDelay(50);
  }
}

void computeFFT(void *parameter){


  while(true){

   while(calcularFFT != 1 || tramaNueva != 1){
      vTaskDelay(50);
   }

    numTramaFFT = numTramaFFT + 2;
    microsecondsFFT = micros();

   for(int i=0; i<samples; i++) //Bucle que lee 1024 muestras de audio
    {
      vReal[i] = wave[i];
      tramaFFT[i] = wave[i];
      vImag[i] = 0;

    }

    arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      //Freq + alta 4482.421875Hz
    double x = FFT.MajorPeak(vReal, samples, samplingFrequency);

    if(x > 85.0 && x < 255.0){
        Serial.println("Voz");
        isVoice = 1;
        noVoiceCounter = 0;
        volumen = computeVolume(tramaFFT);

        /*Serial.println("Quieres recibir volumen?");
        String quieroV = Serial.readString();
        if(quieroV == "1"){
          quieroVolumen = 1;
        }else if(quieroV == "0"){
          quieroVolumen = 0;
        }*/

        if(state_env = VOLUME){
            concat = 1;
            state_env = 1;
            vTaskResume(Enviar);
        }

    }else{
      Serial.println("Otra cosa");
      isVoice = 0;
      noVoiceCounter++;
    }

    //Para de enviar cuando pasan 4 segundos de silencio o maximo 10 segundos de voz
    if(state_env == 3 && (numTramasEnviadas >= tramas10Segundos || noVoiceCounter >= tramas4Segundos)){
      Serial.println("Enviado");
      vTaskSuspend(Enviar);
      noVoiceCounter = 0;
      state_env = 0;
      concat = 0;
      numTramasEnviadas = 0;
    }

    long tiempo = micros() - microsecondsFFT;
    tramaNueva = 0;
    vTaskDelay(50);
  }


}

void enviar(void *parameter){

  while(true){

      switch (state_env){
      case 0:
              // No hacer nada de nada de res de res
        break;
      case 1: { /*Enviar volumen*/
        //Assignar volumen a la variable
        vTaskResume(esp32Client);
        state_env = 2; //error
      }
        break;
      case 2: { /*Esperar*/ //error

        //Serial.println("---->¿Quieres audio?");
        // Esperar respuesta

        if(state_env == WAITRESPONSE){
           state_env = 3;
           //Guardar savedAudio en la variable global
           vTaskResume(esp32Client);
           savedAudio = "";  //Envia el audio que ha ido guardando desde la detección
           numTramasGuardadas = 0;
        }else if(state_env == IDLE){
           state_env = 0;
           concat = 0;
           //Serial.println("No quiero audio");
        }
      }
        break;
      case 3: { /*Enviar data*/
        while( numTramasGuardadas < tramas1Segundo){ //Envia audio de 1 segundo cada vez
          //Empty loop

        }
        numTramasEnviadas += numTramasGuardadas;
        numTramasGuardadas = 0;
        //Guardar savedAudio en la variable global
        //Guardar numTramasEnviadas también?
        vTaskResume(esp32Client);
      }
        break;
      default: {
        state_env = 0;
      }break;

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
                  //Serial.write(c);                    // print it out the serial monitor
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
                        currentLine="";
                        break;

                    }else if (currentLine.endsWith("GET /L ")) {
                        digitalWrite(33, LOW);                // GET /L turns the RELAY off
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        Serial.println("L request detected");
                        client.stop();
                        currentLine="";
                        break;


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
                        currentLine="";
                        break;

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
                        currentLine="";
                        break;

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
                        currentLine="";
                        Serial.print("VOLUME");
                        break;
                    }

                }else{  //End of first request line , no accepted get requested
                    //Error 405 Method Not Allowed
                    client.println("HTTP/1.1 405 Method Not Allowed");
                    client.println();
                    client.stop();
                    currentLine="";
                    break;
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
                        client.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile));
                        if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            state_env=WAITRESPONSE;
                        }
                    break;
                    case AUDIO:
                        client.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile));
                        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            if(EndOfFile==true){
                                EndOfFile=false;
                                //Go back to initial state
                                state_env=IDLE;
                                raspiListening=true;
                            }
                            // We have finished accessing the shared resource.  Release the
                            // semaphore.
                            xSemaphoreGive( dataSemaphore );
                        }
                    break;
                    default:
                        client.println(makeHTTPrequest("POST","/error","application/json",data , localitzationDelta, EndOfFile));
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

    while (status != WL_CONNECTED) {
        Serial.print("Attempting to connect to SSID: ");
        Serial.println(ssid);
        status = WiFi.begin(ssid, password);
        delay(2000);
    }
    Serial.println("Connected");
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(33, LOW);
    delay(1000);
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);
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
        computeFFT,             //Task function
        "FFT",                 //name of task
        1000,                     //Stack size of the task
        NULL,                     //parameter of the task
        1,                        //priority of the task
        &FFT,                   //Task handle to keep track of created task
        0);                       //core

   xTaskCreatePinnedToCore(
        enviar,             //Task function
        "Enviar",                 //name of task
        1000,                     //Stack size of the task
        NULL,                     //parameter of the task
        1,                        //priority of the task
        &Enviar,                   //Task handle to keep track of created task
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

    vTaskSuspend(Enviar);

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
    vTaskDelay(5000);
    Serial.println(state_env);
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
