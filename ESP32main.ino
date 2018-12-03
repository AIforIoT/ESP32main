//NEW ESP32 code

#include <WiFi.h>
#include "arduinoFFT.h"
#include "string.h"

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
    int localitzationDelta=100;
    boolean EndOfFile;


//TASK HANDLES DEFINITIONS
    TaskHandle_t Beacons;
    TaskHandle_t MicroInput;
    TaskHandle_t taskFFT;
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
   // vTaskSuspend(taskFFT);
  while(true){

    /*SAMPLING*/
      microsecondsLectura = micros();
      numTramaLectura++;
      tramaNueva = 0;

     for(int i=0; i<Nwave; i++) //Bucle que lee Nwave muestras de audio
     {

        //Serial.println("Proceso 2");
        wave[i] = analogRead(CHANNEL); // Lectura del valor del pin
       // Serial.println("---------------->>wave[" + (String)i + "] (" + (String)numTramaLectura +") = " + (String)wave[i]);
        waveString += (String)wave[i];
        waveString += ",";//AMB COMA??

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
      //Serial.println("Concatenando...");
      numTramasGuardadas++;
     }




        tramaNueva = 1;
        calcularFFT = !calcularFFT;

        vTaskDelay(1);

        if(tramaNueva == 1 && calcularFFT == 1){
          vTaskResume(taskFFT);

        }


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
      vReal[i] = waveForFFT[i];
      //Serial.println("-----------------> vReal[" +(String)i + "] (" + (String)numTramaFFT + ") = " + (String)vReal[i]);
      tramaFFT[i] = waveForFFT[i];
      vImag[i] = 0;

    }

    arduinoFFT FFT = arduinoFFT(); /* Create FFT object */

    FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      //Freq + alta 4482.421875Hz
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

    if(localState == IDLE && isVoice == 1 && localRaspiListening==1){

      concat = 1;

      if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
      {
          // We were able to obtain the semaphore and can now access the
          // shared resource.
          data = (String)volumen;
          // We have finished accessing the shared resource.  Release the
          // semaphore.
          xSemaphoreGive( dataSemaphore );
      }
      if( xSemaphoreTake( stateSemaphore, portMAX_DELAY ) == pdTRUE )
      {
          // We were able to obtain the semaphore and can now access the
          // shared resource.
          state_env = VOLUME;
          // We have finished accessing the shared resource.  Release the
          // semaphore.
          xSemaphoreGive( stateSemaphore );
      }

      //asignar con semaforo
      vTaskResume(esp32Client);
    }


    //Para de enviar cuando pasan 4 segundos de silencio o maximo 10 segundos de

    if(localState == AUDIO){
    /*  if( numTramasGuardadas == tramas1Segundo){ //Envia audio de 1 segundo cada vez

        //envia audio

        if( xSemaphoreTake( dataSemaphore, portMAX_DELAY ) == pdTRUE )
        {
            // We were able to obtain the semaphore and can now access the
            // shared resource.
            data = savedAudio;
            // We have finished accessing the shared resource.  Release the
            // semaphore.
            xSemaphoreGive( dataSemaphore );
        }

        //asignar con semaforo
        vTaskResume(esp32Client);


        numTramasEnviadas += numTramasGuardadas;
        numTramasGuardadas = 0;
      }else if*/
      if((numTramasEnviadas >= tramas10Segundos || noVoiceCounter >= tramas4Segundos)){
        //Serial.println("Enviado");

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
        noVoiceCounter = 0;
        concat = 0;
        numTramasEnviadas = 0;
      }

    }


    long tiempo = micros() - microsecondsFFT;


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
                        client.println(makeHTTPrequest("POST","/volume","application/json",data, localitzationDelta ,EndOfFile, esp_ip));
                        state_env=WAITRESPONSE;
                    break;
                    case AUDIO:
                        client.println(makeHTTPrequest("POST","/audio","application/json",data , localitzationDelta, EndOfFile, esp_ip));
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
                        client.println(makeHTTPrequest("POST","/error","application/json",data , localitzationDelta, EndOfFile, esp_ip));
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
    pinMode(33, OUTPUT);       // set the RELAY pin mode
    pinMode(32, OUTPUT);      // set the RELAY pin mode
    Serial.begin(115200);
    sampling_period_us = round(1000000*(1.0/samplingFrequency));

    vSemaphoreCreateBinary( dataSemaphore );
    vSemaphoreCreateBinary( stateSemaphore );
    vSemaphoreCreateBinary( fftSemaphore );

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
                    //Parse PASS
                    int initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    password=postBody.substring(initIndex+1,lastIndex);
                    //Parse TYPE
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    type=postBody.substring(initIndex+1,lastIndex);
                    //Parse RASPIP
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    raspip=postBody.substring(initIndex+1,lastIndex);
                    //Parse xPos
                    initIndex=postBody.indexOf("=",lastIndex);
                    lastIndex=postBody.indexOf("&",lastIndex+1);
                    xPos=postBody.substring(initIndex+1,lastIndex);
                    //Parse yPos
                    initIndex=postBody.indexOf("=",lastIndex);
                    yPos=postBody.substring(initIndex+1);
                    //Parse location
                    initIndex=postBody.indexOf("=",lastIndex);
                    side=postBody.substring(initIndex+1);
                    initIndex=postBody.indexOf("=",lastIndex);
                    location=postBody.substring(initIndex+1);
                    requestDetected=NONE;
                    currentLine="";
                    client.stop();

                    //Now we have the login information we try to connect to raspi.
                    Serial.print("Attempting to connect to SSID: ");
                    Serial.println(ssid);
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
        }//END WHILE

        //Connected to system WIFI, begin RASPI handshke:
        //if there's a successful connection:
        if (client.connect(raspip.c_str(), 8080)) {
            Serial.println("connecting...");
            // send the HTTP GET request:
            client.println("POST /setUp HTTP/1.1");
            client.println("content-type: application/json");
            client.println("");
            client.println("\"esp_id\": \""+esp_ip+"\",");
            client.println("\"esp_ip\": \""+esp_ip+"\",");
            client.println("\"esp_type\": \""+type+"\",");
            client.println("\"esp_x_axis\": \""+xPos+"\",");
            client.println("\"esp_y_axis\": \""+yPos+"\"");
            client.println("\"side\": \""+side+"\",");
            client.println("\"location\": \""+location+"\",");
            client.println("");
            Serial.println("Handshake sent, wainting for confirmation...");
            // listen for incoming clients
            server.begin();
            WiFiClient raspi = server.available();
            Serial.println("Server started");
            Serial.print("Server is at ");
            Serial.println(WiFi.localIP());
            whileExit=false;
            char c;
            while(true)
            {
                if (raspi) {
                    Serial.println("new client");
                    String currentLine = "";                // make a String to hold incoming data from the client

                    while (raspi.connected()) {            // loop while the client's connected
                        if (raspi.available()) {             // if there's bytes to read from the client,
                            c = raspi.read();             // read a byte, then
                            Serial.println(
                        }
                        //This ESP32 server only expects HTTP Request:
                        // GET 200 HTTP/1.x
                        // GET 500 HTTP/1.x
                        if (c != '\r') {  // if you got anything else but a carriage return character,
                            currentLine += c;      // add it to the end of the currentLine
                            // Check to see if the client request was "GET /H" or "GET /L":
                            if (currentLine.endsWith("GET 200")) {
                                raspi.println("HTTP/1.1 200 OK");
                                raspi.println();
                                raspi.stop();
                                currentLine="";
                                whileExit=true;
                                break;

                            }else if (currentLine.endsWith("GET 500 ")) {
                                raspi.println("HTTP/1.1 200 OK");
                                raspi.println();
                                Serial.println("FATAL ERROR: Unable to handshake with Raspberry Pi.");
                                Serial.println("RESTARTING ESP32 CONFIG");
                                raspi.stop();
                                currentLine="";
                                loginERROR=true;
                                whileExit=true;
                                break;
                            }
                        }
                    }
                }
                if(whileExit==true)
                {
                    break;
                }
            }
        }else{
            // if you couldn't make a connection:
            Serial.println("FATAL ERROR: Unable to handshake with Raspberry Pi.");
            Serial.println("RESTARTING ESP32 CONFIG");
            loginERROR=true;
        }
        if(loginERROR==false){
            break; //EXIT LOGIN WHILE
        }
    }//END OF LOGIN WHILE

    Serial.println("Connected");

    //HARDWARE CHECK
    digitalWrite(33, HIGH);
    delay(1000);
    digitalWrite(33, LOW);
    delay(1000);
    digitalWrite(32, HIGH);
    delay(1000);
    digitalWrite(32, LOW);
    delay(1000);

    //THREAD INIT
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

}
//DO NOTHING IN LOOP
void loop(){
    // send it out the serial port.This is for debugging purposes only:
    vTaskDelay(5000);
    Serial.println(state_env);
}

//Auxiliary functions
String makeHTTPrequest(String method, String uri, String type, String data, int localitzationDelta, boolean EndOfFile, String esp_ip){
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
    if(uri=="/volume")
    postBody=postBody+
    "{\n"
    "\"esp_id\": \""+esp_ip+"\",\n"
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
