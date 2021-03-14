import processing.serial.*;
import controlP5.*;

ControlP5 gui;
Serial activePort;

void setup() {
  size(800, 400);
  noStroke();
  
  gui = new ControlP5(this);
  
  gui.addScrollableList("serialPortPicker")
    .setLabel("Serial Port")
    .setType(ControlP5.DROPDOWN)
    .setPosition(20, 20)
    .setSize(200, 100)
    .setBarHeight(20)
    .setItemHeight(20)
    .addItems(Serial.list())
    .close();
    
  gui.addButton("connectButton")
    .setLabel("Connect")
    .setPosition(230, 20)
    .setSize(100, 20)
    .onRelease(new CallbackListener() {
        public void controlEvent(CallbackEvent e) {
          if (activePort != null) {
            disconnect();
            return;
          }
          
          String selectedPort = gui.get(ScrollableList.class, "serialPortPicker").getCaptionLabel().getText();
          
          if (selectedPort == "Serial Port") {
            return;
          }
          
          connectToSerialPort(selectedPort);
        }
    });
}

void disconnect() {
  if (activePort == null) return;
  
  gui.get(Button.class, "connectButton")
    .setLabel("Connect");
    
  gui.get(ScrollableList.class, "serialPortPicker").unlock();
    
  activePort.stop();
  activePort = null;
}

void connectToSerialPort(String port) {
  gui.get(Button.class, "connectButton")
    .setLabel("Connecting...")
    .lock();
    
  activePort = new Serial(this, port, 9600);
  
  if (activePort != null) {
    gui.get(Button.class, "connectButton")
      .setLabel("Disconnect")
      .unlock();
    
    gui.get(ScrollableList.class, "serialPortPicker").lock();
  } else {
    gui.get(Button.class, "connectButton")
      .setLabel("Connect")
      .unlock();
  }
}

void draw() {
  background(color(240, 240, 240));
}
