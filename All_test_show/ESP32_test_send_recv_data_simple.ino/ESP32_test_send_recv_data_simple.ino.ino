#include <WiFi.h>

const char* ssid = "laptophuy";
const char* password = "huy123qwe";

const char* serverAddress = "192.168.137.1";
const int serverPort = 1234; // Change this to your desired port number

WiFiClient client;

float position = 3.0; // Example position data
float velocity = 2.0; // Example velocity data

void setup() {
  Serial.begin(115200);
  delay(100);

  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  client.connect(serverAddress, serverPort);
  delay(200);


  // ESP32 client to wait for the "OK" response in the setup function before proceeding
  while (!client.connected()) {
    if (!client.connect(serverAddress, serverPort)) {
      Serial.println("Connection failed.");
    }
  }


  // ESP32 send a first message to server know which device is connected 
  Serial.println("Data send : esp32 sended");
  client.print("esp32 sended");
  Serial.println("Wait server response");
  // delay(20);

  while (client.connected()) {
    String response = client.readStringUntil('\n');
    Serial.println(response);
  }
  
  Serial.println("Loss connection to server");
    
}

// void dataTransmission() {

//     // Send the data as a string to the server
//     String dataString = String(position) + "," + String(velocity);
//     client.print(dataString);

//     Serial.print("Sent data: ");
//     Serial.println(dataString);

//   }
// }


// void dataAcquisitionTask(void *pvParameters) {
//   while (!client.connected()) {
//     if (!client.connect(serverAddress, serverPort)) {
//       Serial.println("Connection failed.");
//       delay(20);
//     }
//   }

//   String response = client.readStringUntil('\n');
//   Serial.println("Server response: " + response);
// }

void loop() {
}
