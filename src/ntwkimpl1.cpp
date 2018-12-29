// /*
//  UDPSendReceiveString:
//  This sketch receives UDP message strings, prints them to the serial port
//  and sends an "acknowledge" string back to the sender
//
//  A Processing sketch is included at the end of file that can be used to send
//  and received messages for testing with a computer.
//
//  created 21 Aug 2010
//  by Michael Margolis
//
//  Web Server
//
// A simple web server that shows the value of the analog input pins.
// using an Arduino Wiznet Ethernet shield.
//
// Circuit:
// * Ethernet shield attached to pins 10, 11, 12, 13
// * Analog inputs attached to pins A0 through A5 (optional)
//
//  This code is in the public domain.
//  */
//
// #include <SPI.h>
// #include <Ethernet.h>
// #include <EthernetUdp.h>
//
// // Enter a MAC address and IP address for your controller below.
// // The IP address will be dependent on your local network:
// byte mac[] = {
//   0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED
// };
// IPAddress ip(192, 168, 1, 177);
//
// unsigned int localPort = 8888;      // local port to listen on
//
// // Initialize the Ethernet server library
// // with the IP address and port you want to use
// // (port 80 is default for HTTP):
// EthernetServer server(80);
//
// // buffers for receiving and sending data
// char packetBuffer[UDP_TX_PACKET_MAX_SIZE];  // buffer to hold incoming packet,
// char ReplyBuffer[] = "acknowledged";        // a string to send back
//
// // An EthernetUDP instance to let us send and receive packets over UDP
// EthernetUDP Udp;
//
// void setup() {
//   // You can use Ethernet.init(pin) to configure the CS pin
//   //Ethernet.init(10);  // Most Arduino shields
//   //Ethernet.init(5);   // MKR ETH shield
//   //Ethernet.init(0);   // Teensy 2.0
//   //Ethernet.init(20);  // Teensy++ 2.0
//   //Ethernet.init(15);  // ESP8266 with Adafruit Featherwing Ethernet
//   //Ethernet.init(33);  // ESP32 with Adafruit Featherwing Ethernet
//
//   // start the Ethernet
//   Ethernet.begin(mac, ip);
//
//   // Open serial communications and wait for port to open:
//   Serial.begin(9600);
//
//   Serial.println("Ethernet WebServer Example");
//
//   // Check for Ethernet hardware present
//   if (Ethernet.hardwareStatus() == EthernetNoHardware) {
//     Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
//     while (true) {
//       delay(1); // do nothing, no point running without Ethernet hardware
//     }
//   }
//   if (Ethernet.linkStatus() == LinkOFF) {
//     Serial.println("Ethernet cable is not connected.");
//   }
//
//   // start UDP
//   Udp.begin(localPort);
//
//   // start the server
//   server.begin();
//   Serial.print("server is at ");
//   Serial.println(Ethernet.localIP());
// }
//
// void loop() {
//   // if there's data available, read a packet
//   int packetSize = Udp.parsePacket();
//   if (packetSize) {
//     Serial.print("Received packet of size ");
//     Serial.println(packetSize);
//     Serial.print("From ");
//     IPAddress remote = Udp.remoteIP();
//     for (int i=0; i < 4; i++) {
//       Serial.print(remote[i], DEC);
//       if (i < 3) {
//         Serial.print(".");
//       }
//     }
//     Serial.print(", port ");
//     Serial.println(Udp.remotePort());
//
//     // read the packet into packetBufffer
//     Udp.read(packetBuffer, UDP_TX_PACKET_MAX_SIZE);
//     Serial.println("Contents:");
//     Serial.println(packetBuffer);
//
//     // send a reply to the IP address and port that sent us the packet we received
//     Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
//     Udp.write(ReplyBuffer);
//     Udp.endPacket();
//   }
//   delay(10);
//
//   // web server main
//   // listen for incoming clients
//   EthernetClient client = server.available();
//   if (client) {
//     Serial.println("new client");
//     // an http request ends with a blank line
//     bool currentLineIsBlank = true;
//     while (client.connected()) {
//       if (client.available()) {
//         char c = client.read();
//         Serial.write(c);
//         // if you've gotten to the end of the line (received a newline
//         // character) and the line is blank, the http request has ended,
//         // so you can send a reply
//         if (c == '\n' && currentLineIsBlank) {
//           // send a standard http response header
//           client.println("HTTP/1.1 200 OK");
//           client.println("Content-Type: text/html");
//           client.println("Connection: close");  // the connection will be closed after completion of the response
//           client.println("Refresh: 5");  // refresh the page automatically every 5 sec
//           client.println();
//           client.println("<!DOCTYPE HTML>");
//           client.println("<html>");
//           // output the value of each analog input pin
//           for (int analogChannel = 0; analogChannel < 6; analogChannel++) {
//             int sensorReading = analogRead(analogChannel);
//             client.print("analog input ");
//             client.print(analogChannel);
//             client.print(" is ");
//             client.print(sensorReading);
//             client.println("<br />");
//           }
//           client.println("</html>");
//           break;
//         }
//         if (c == '\n') {
//           // you're starting a new line
//           currentLineIsBlank = true;
//         } else if (c != '\r') {
//           // you've gotten a character on the current line
//           currentLineIsBlank = false;
//         }
//       }
//     }
//     // give the web browser time to receive the data
//     delay(1);
//     // close the connection:
//     client.stop();
//     Serial.println("client disconnected");
//   }
// }
//
//
// /*
//   Processing sketch to run with this example
//  =====================================================
//
//  // Processing UDP example to send and receive string data from Arduino
//  // press any key to send the "Hello Arduino" message
//
//
//  import hypermedia.net.*;
//
//  UDP udp;  // define the UDP object
//
//
//  void setup() {
//  udp = new UDP( this, 6000 );  // create a new datagram connection on port 6000
//  //udp.log( true );         // <-- printout the connection activity
//  udp.listen( true );           // and wait for incoming message
//  }
//
//  void draw()
//  {
//  }
//
//  void keyPressed() {
//  String ip       = "192.168.1.177"; // the remote IP address
//  int port        = 8888;        // the destination port
//
//  udp.send("Hello World", ip, port );   // the message to send
//
//  }
//
//  void receive( byte[] data ) {          // <-- default handler
//  //void receive( byte[] data, String ip, int port ) {   // <-- extended handler
//
//  for(int i=0; i < data.length; i++)
//  print(char(data[i]));
//  println();
//  }
//  */
