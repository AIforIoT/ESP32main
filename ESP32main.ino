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
#include "AsyncUDP.h"
#include "esp32-hal-timer.h"

//WIFI DEFINITIONS

    int status = WL_IDLE_STATUS;


//    const char* ssid     =    "iouti_net";
//    const char* password =    "thenightmareofhackers";
//    const char* raspip =      "192.168.5.1";
//    const char* ssid     =    "Aquaris X5 Plus";
//    const char* password =    "3cdb401cb5d6";
//    const char* raspip =      "192.168.43.81";

    const String HTMLPage=""
    "<!DOCTYPE html>\n"
    "<html>\n"
    "<head>\n"
    "<style>\n"
    "    h1 {text-align:center;}\n"
    "    p {text-align:center;}\n"
    "    fieldset {\n"
    "        text-align:center;\n"
    "        display: block;\n"
    "        margin-left: 100px;\n"
    "        margin-right: 100px;\n"
    "        padding-top: 0.35em;\n"
    "        padding-bottom: 0.625em;\n"
    "        padding-left: 0.75em;\n"
    "        padding-right: 0.75em;\n"
    "        border: 2px groove (internal value);\n"
    "    }\n"
    "    form {text-align:center;}\n"
    "    input[type=text] {\n"
    "        margin: 8px 0;\n"
    "        box-sizing: border-box;\n"
    "        border: 3px solid #ccc;\n"
    "        -webkit-transition: 0.5s;\n"
    "        transition: 0.5s;\n"
    "        outline: none;\n"
    "    }\n"
    "    input[type=text]:focus {\n"
    "        border: 3px solid #555;\n"
    "    }\n"
    "    input[type=password] {\n"
    "        margin: 8px 0;\n"
    "        box-sizing: border-box;\n"
    "        border: 3px solid #ccc;\n"
    "        -webkit-transition: 0.5s;\n"
    "        transition: 0.5s;\n"
    "        outline: none;\n"
    "    }\n"
    "    input[type=password]:focus {\n"
    "        border: 3px solid #555;\n"
    "    }\n"
    "    select{\n"
    "        border: 3px solid #ccc;\n"
    "    }\n"
    "    select:focus{\n"
    "        border: 3px solid #555;\n"
    "    }\n"
    "    body.sansserif {\n"
    "        font-family: Arial, Helvetica, sans-serif;\n"
    "    }\n"
    "</style>\n"
    "</head>\n"
    "<body class=\"sansserif\">\n"
    "\n"
    "    <h1>Project \"IOUTI\"</h1>\n"
    "\n"
    "    <p>Login WebPage for your ESP32</p>\n"
    "    <fieldset>\n"
    "        <form method=\"post\">\n"
    "            \n"
    "            SSID:<br>\n"
    "            <input type=\"text\" name=\"SSID\"><br>\n"
    "            PASSWORD:<br>\n"
    "            <input type=\"password\" name=\"PASS\"><br>\n"
    "            ESP32 type:<br><br>\n"
    "            <select name=\"type\">\n"
    "                <option value=\"plug\" selected>Plug</option>\n"
    "                <option value=\"light\">Light</option>\n"
    "                <option value=\"blind\">Blind</option>\n"
    "                <option value=\"Example2\">Ipsum Lorem</option>\n"
    "              </select>\n"
    "              <br><br>\n"
    "            Raspi IP:<br>\n"
    "            <input type=\"text\" name=\"RASPIP\"><br>\n"
    "\n"
    "            ESP32 X axis:<br>\n"
    "            <input type=\"text\" name=\"XAXIS\"><br>\n"
    "            ESP32 Y axis:<br>\n"
    "            <input type=\"text\" name=\"YAXIS\"><br>\n"
    "            ESP32 Location:<br>\n"
    "            Side:\n"
    "            <select name=\"side\">\n"
    "                <option value=\"none\" selected>None</option>\n"
    "                <option value=\"left\">Left</option>\n"
    "                <option value=\"right\">Right</option>\n"
    "            </select><br>\n"
    "            Location:\n"
    "            <select name=\"location\">\n"
    "                <option value=\"none\" selected>None</option>\n"
    "                <option value=\"table\">Table</option>\n"
    "                <option value=\"corner\">Corner</option>\n"
    "                <option value=\"wall\">Wall</option>\n"
    "            </select>\n"
    "            <br><br>\n"
    "\n"
    "            <input type=\"submit\" value=\"Submit\">\n"
    "        </form>\n"
    "    </fieldset>\n"
    "</body>\n"
    "</html>\n"
    "";

//LOGIN ESP DEFINITIONS
    const int NONE=0;
    const int GET=1;
    const int POST=2;
    const int port = 8080;
    String ssid     = "";
    String password = "";
    String type     = "";
    String raspip   = "";
    String xPos     = "";
    String yPos     = "";
    String location = "";
    String side     = "";
    String esp_ip   = "";


//INPUT DEFINITIONS
    #define CHANNEL 36
    #define SCL_INDEX 0x00
    #define SCL_TIME 0x01
    #define SCL_FREQUENCY 0x02
    #define SCL_PLOT 0x03
    const int Nwave = 1024; //This value MUST ALWAYS be a power of 2
    const double samplingFrequency = 16000; //Hz

    //INPUT VARIABLES
    boolean isVoice = 0;
    volatile boolean concat = 0;
    volatile boolean newAudio = 1;
    double volume;
    volatile int numTramasGuardadas = 0;
    int tramas10Segundos = 45; //156;
    int numTramasEnviadas;
    int numTramasTotal;
    int framesPacket = 3;
    double volumeThreshold = 700.0;
    int bufferPrevSize = 4; //En tramas
    float zeros = 0;

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


//WIFI INIT

    // Initialize the Wifi client library
    WiFiClient client;
    // Initialize the Wifi server library
    WiFiServer server(80);

//UDP Server

  AsyncUDP udp;


//Timer isr

  hw_timer_t *timer = NULL;

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


String uint64toString(uint64_t num){
    uint32_t low = num % 0xFFFFFFFF;
    uint32_t high = (num >> 32) % 0xFFFFFFFF;
    return String(low)+String(high);
}
void IRAM_ATTR onTimer(){
  Serial.println("Missed N synchronizing frames... Problem there?");
}
void codeForBeacons( void * parameter){
    Serial.begin(115200);
    if(udp.listen(IPAddress(0,0,0,0),9013)) {
      Serial.print("UDP connected listenning on IP:");
      Serial.print(WiFi.localIP());
      Serial.println(WiFi.subnetMask());

      udp.onPacket([](AsyncUDPPacket packet) {

//        Serial.print("UDP Packet Type: ");
//        Serial.print(packet.isBroadcast()?"Broadcast":packet.isMulticast()?"Multicast":"Unicast");
//        Serial.print(", From: ");
//        Serial.print(packet.remoteIP());
//        Serial.print(":");
//        Serial.print(packet.remotePort());
//        Serial.print(", To: ");
//        Serial.print(packet.localIP());
//        Serial.print(":");
//        Serial.print(packet.localPort());
//        Serial.print(", Length: ");
//        Serial.print(packet.length());
//        Serial.print(", Data: ");
//        Serial.write(packet.data(), packet.length());
//        Serial.println();
        //reply to the client

        if(packet.remoteIP().toString()==raspip){
            Serial.print("Read from timer:");
            uint64_t timer_count = getDelayLocalization();
            uint32_t low = timer_count % 0xFFFFFFFF;
            uint32_t high = (timer_count >> 32) % 0xFFFFFFFF;
            reinitializeTimer();
            Serial.print(low);
            Serial.println(high);
            packet.printf("Got %u bytes of data", packet.length());

        }

      });
    }
}
uint64_t getDelayLocalization(){
  return timerRead(timer);
}
void reinitializeTimer(){
  timerWrite(timer, 0); //reset timer (feed watchdog)
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
                numTramasGuardadas = (savedAudio.length())/(2*1024);
                waveString = "";                
                Serial.println("---------------------------------------ESPERA...");
            }
        }else if(!concat && !newAudio){ //To not concatenate or save previous audio
           //BUIT
        }else if(concat){  //To concatenate certain seconds of audio while sending it
           //Concantenar el audio que vas recibiendo
           savedAudio += waveString;
           waveString = "";
           //Serial.println("Concatenando...");
           numTramasGuardadas = (savedAudio.length())/(2*1024);
           numTramasTotal = (savedAudio.length())/(2*1024);
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
    zeros = zcr(tramaFFT, Nwave);

    if((x > 85.0 && x < 255.0) && (zeros <= 300)){ //x is the peak frequency, it detects voice if this value is between 85 Hz and 255 Hz
        //Serial.println("-----------------------Voz");
        isVoice = 1;
        volume = computeVolume(tramaFFT);
//        Serial.println(volume);
    }else{
      isVoice = 0;
    }


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

    if((numTramasTotal >= tramas10Segundos)){
       newAudio = 0;
       concat = 0;             
    }

    if(localState == IDLE && localRaspiListening == 0){
        concat = 0;
        numTramasGuardadas = 0;
    }else if(localState == IDLE && isVoice == 1 && localRaspiListening == 1 && savedAudio.length() >= bufferPrevSize*2*1024 && volume >= volumeThreshold){ 
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
            data = (String)volume;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }
    
        //asignar con semaforo
        Serial.println("-------------------------------------------->Petición");
        Serial.println((String)volume);
        //Serial.println(savedAudio.length());
        vTaskResume(esp32Client);

    } else if(localState == AUDIO){

      if(savedAudio.length() <= framesPacket*2*1024){ //When there is less than 6*1024 characters in savedAudio, it is sent with the End of File

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
  
        Serial.print("Tramas Enviadas: ");
        Serial.println(numTramasEnviadas);
        vTaskResume(esp32Client);
        Serial.println("Enviado");
  
        /*Reiniciamos variables*/
        savedAudio= "";
        newAudio = 1;
        concat = 0;
        numTramasEnviadas = 0;
        numTramasGuardadas = 0;
        numTramasTotal = 0;

      } else { 

       int indFin = Nwave*framesPacket*2; //3 tramas
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
         Serial.print("Tramas Enviadas: ");
         Serial.println(numTramasEnviadas);
         Serial.print("Tramas Guardadas: ");
         Serial.print(numTramasGuardadas);
         Serial.print("-->");
         Serial.println(savedAudio.length());
         Serial.print("-->");
         Serial.println(data.length());
         Serial.print("Tramas Total: ");
         Serial.println(numTramasTotal);
         /**/

         numTramasEnviadas += framesPacket;
         numTramasGuardadas -= framesPacket;

         //Serial.println("enviando...");
    }
  }
  vTaskDelay(300);
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
                        digitalWrite(19, HIGH);               // GET /H turns the REALY on
                        client.println("HTTP/1.1 200 OK");
                        client.println();
                        //Serial.println("H request detected");
                        client.stop();
                        currentLine="";
                        break;

                    }else if (currentLine.endsWith("GET /L ")) {
                        digitalWrite(19, LOW);                // GET /L turns the RELAY off
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
                            // shared resource
                            if(state_env==WAITRESPONSE){
                                state_env=IDLE;
                            }
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
        if (client.connect(raspip.c_str(), port)) {
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
                        client.println(makeHTTPrequest("POST","/volume","application/json",data, getDelayLocalization() ,EndOfFile, esp_ip));
                        state_env=WAITRESPONSE;
                    break;
                    case AUDIO:
                        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
                        {
                            // We were able to obtain the semaphore and can now access the
                            // shared resource.
                            client.println(makeHTTPaudio(esp_ip,data,EndOfFile));
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
                        client.println(makeHTTPrequest("POST","/error","application/json",data , getDelayLocalization(), EndOfFile, esp_ip));
                        state_env=IDLE;
                        raspiListening=false;
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

    //***********************************************
    //                  LOGIN CODE
    //***********************************************
    while(true){

        WiFi.softAP("ESP32_IOT");
        Serial.println("ESP32 acces point started");
        Serial.print("Server is at: ");
        Serial.println(WiFi.softAPIP());
        server.begin();
        boolean flag=false;
        boolean loginERROR=false;
        boolean whileExit=false;
        String postBody="";                     //Make a String to save post body data
        int contentLenght=0;
        int index=0;
        int requestDetected=NONE;               //Flag that shows if the request has been detected
                                                //- 0 : Not detected
                                                //- 1 : GET detected
                                                //- 2 : POST detected
        String currentLine = "";                // make a String to hold incoming data from the client

        //This while contains the necessary code to initialize wifi config
        while(true){
            // listen for incoming clients
            WiFiClient client = server.available();
            whileExit=false;
            if (client) {
                Serial.println("new client");
                while (client.connected()&& whileExit != true) {            // loop while the client's connected
                    if (client.available()) {             // if there's bytes to read from the client,
                        char c = client.read();             // read a byte, then
                        Serial.print(c);                    // print it out the serial monitor
                        currentLine=currentLine+c;

                        if(requestDetected==NONE){
                            if(currentLine.startsWith("GET")){
                                requestDetected=GET;
                            }
                            if(currentLine.startsWith("POST")){
                                requestDetected=POST;
                            }
                        }
                    }
                    switch (requestDetected) {
                        case NONE:     //RESQUEST NOT DETECTED
                            //??
                        break;
                        case GET:     //GET DETECTED
                            if(currentLine.endsWith("\r\n\r\n")){
                                if(flag==false){
                                    flag=true;
                                }else{
                                    flag=false;
                                    Serial.println("END REQUEST");
                                    whileExit=true;
                                }
                            }
                        break;
                        case POST:     //POST DETECTED
                            if(currentLine.endsWith("\r\n") && contentLenght== 0){
                                index=currentLine.indexOf("Content-Length: ");
                                if(index >=0){
                                    contentLenght= currentLine.substring(index+16).toInt();
                                }
                            }
                            //Gets post request body Information
                            if(currentLine.indexOf("\r\n\r\n")>0){
                                contentLenght--;
                                if(contentLenght<0){
                                    postBody=currentLine.substring(currentLine.indexOf("\r\n\r\n"));
                                    contentLenght=0;
                                    whileExit=true;
                                    Serial.println("END POST");
                                }
                            }
                        break;
                    }//END switch
                }//END While client connected

                // Check to see if the client request was "GET" or "POST":
                if (requestDetected==POST) {
                    //CAUTION, PASSWORD NOT ENCRYPTED
                    Serial.println("POST REQUEST");
                    String httpResponse = "";
                    httpResponse += "HTTP/1.1 200 OK\r\n";
                    httpResponse += "Content-type:text/html\r\n\r\n";
                    httpResponse += "<h1>POST OK</h1>";
                    httpResponse += "\r\n";

                    client.println(httpResponse);
                    //Example of body of POST: SSID=TEST&PASS=123456789&type=light
                    //Parse SSID
                    int lastIndex=postBody.indexOf("&");
                    ssid=postBody.substring(postBody.indexOf("=")+1,lastIndex);
                    ssid.replace('+',' ');
                    Serial.println(ssid);
                    //Parse PASS
                    int initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    password=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(password);
                    //Parse TYPE
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    type=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(type);
                    //Parse RASPIP
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    raspip=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(raspip);
                    //Parse xPos
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    xPos=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(xPos);
                    //Parse yPos
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    yPos=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(yPos);
                    //Parse side
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    side=postBody.substring(initIndex+1,lastIndex);
                    Serial.println(side);
                    //Parse location
                    initIndex=postBody.indexOf("=",lastIndex);
                    location=postBody.substring(initIndex+1);
                    Serial.println(location);
                    requestDetected=NONE;
                    currentLine="";
                    client.stop();

                    //Now we have the login information we try to connect to raspi.
                    Serial.print("Attempting to connect to SSID: ");
                    Serial.println(ssid);
                    WiFi.mode(WIFI_STA);
                    WiFi.begin(ssid.c_str(), password.c_str());
                    int count=20;
                    while (WiFi.status() != WL_CONNECTED && count>=0) {
                        delay(500);
                        Serial.print(".");
                        count--;
                    }
                    Serial.println(" ");
                    if(WiFi.status() == WL_CONNECTED){
                        Serial.println("WiFi connected");
                        Serial.println("IP address: ");
                        Serial.println(WiFi.localIP());
                        esp_ip=WiFi.localIP().toString();
                        break;
                    }else{
                        Serial.println("Unable to connect to: ");
                        Serial.println(ssid);
                        Serial.println("FATAL ERROR: RESTARTING ESP32 CONFIG");
                    }
                }//END of POST
                if (requestDetected==GET) {
                    // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
                    // and a content-type so the client knows what's coming, then a blank line:
                    Serial.println("GET REQUEST");
                    String httpResponse = "";
                    httpResponse += "HTTP/1.1 200 OK\r\n";
                    httpResponse += "Content-type:text/html\r\n\r\n";

                    // then the HTML page
                    httpResponse += HTMLPage;

                    // The HTTP response ends with a blank line:
                    httpResponse += "\r\n";
                    client.println(httpResponse);
                    currentLine="";
                    requestDetected=NONE;
                    client.stop();
                    Serial.println("Client Disconnected.");
                }//END of GET
            }//END if client
        }//END WEB PAGE SERVER WHILE

        //Connected to system WIFI, begin RASPI handshke:
        //if there's a successful connection:
        if (client.connect(raspip.c_str(), 8080)) {
            //Serial.println("connecting...");

            // send the HTTP GET request:
            String dataToSend = "{\n"
                                "\"esp_id\": \""+esp_ip+"\",\n"
                                "\"esp_ip\": \""+esp_ip+"\",\n"
                                "\"esp_type\": \""+type+"\",\n"
                                "\"esp_x_axis\": \""+xPos+"\",\n"
                                "\"esp_y_axis\": \""+yPos+"\",\n"
                                "\"side\": \""+side+"\",\n"
                                "\"location\": \""+location+"\"\n"
                                "}\n"
                                "\n";

            String header = "POST /setUp HTTP/1.1\n"
                            "content-type: application/json\n"
                            "content-Length: "+ String(dataToSend.length())+"\n\n";


            client.println(header+dataToSend);
            Serial.println(header+dataToSend);
            break;

        }else{
            // if you couldn't make a connection:
            Serial.println("FATAL ERROR: Unable to handshake with Raspberry Pi.");
            Serial.println("RESTARTING WIFI ESP32 CONFIG");
        }

    }//END OF LOGIN WHILE
    Serial.println("SUCCES: Handshake completed");
    Serial.println("*************************");
    Serial.println("Connected to IOT SYSTEM !");
    Serial.println("Project IOUTI");
    Serial.println("PAE UPC 2018");
    Serial.println("*************************");
    Serial.println("INIT sequence start");

    //HARDWARE CHECK
    digitalWrite(19, HIGH);
    delay(1000);
    digitalWrite(19, LOW);
    delay(1000);
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);

    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &onTimer, true);
    timerAlarmWrite(timer, 5000000, true);
    timerAlarmEnable(timer);

    codeForBeacons(NULL);

    //THREAD INIT

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
        1);                       //core


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
    vTaskDelay(2000);
    Serial.println("----------------------------------->" + (String)state_env);
}

//Auxiliary functions
String makeHTTPrequest(String method, String uri, String type, String data, uint64_t localitzationDelta, boolean EndOfFile, String esp_ip){
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
        dataToSend = data; //volume is a int value in state AUDIO.
        localitzationToSend = uint64toString(localitzationDelta);
        EOFtoSend=EndOfFile;
        // We have finished accessing the shared resource.  Release the
        // semaphore.
        xSemaphoreGive( dataSemaphore );
    }
    if(localitzationToSend=="00"){
      localitzationToSend="0";
    }
    postBody=postBody+
    "{\n"
    " \"esp_id\": \""+esp_ip+"\",\n"
    " \"timestamp\": \""+EOFtoSend+"\",\n"
    " \"delay\": \""+ localitzationToSend +"\",\n"
    " \"volume\": \""+dataToSend+"\"\n"
    "}\n";

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

float zcr(const double *x, unsigned int N){
  float cont = 0;
  int i;
  for (i = 0; i <= (N-1); i++){
    if(x[i-1]*x[i]<0){
      cont++;
    }
  }
  return cont;
}

String makeHTTPaudio(String esp_ip, String postBody, boolean EndOfFile){
    String header="POST /audio/";
    header=header+esp_ip+"/"+String(EndOfFile)+" HTTP/1.1\n"+"content-type: text/plain\n"+"content-Length: "+String(postBody.length())+"\n\n";
    return header+postBody;
}
