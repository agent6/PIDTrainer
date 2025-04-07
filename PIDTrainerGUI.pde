import processing.serial.*;
import controlP5.*;

Serial myPort;
ControlP5 cp5;
DropdownList portList;

boolean pidEnabled = false;
float baselineTime = -1;  // Baseline timestamp (ms) when PID is enabled

// List to store graph points: x = relative time (sec), y = encoder value (ticks)
ArrayList<PVector> graphPoints;

String receivedLine = "";

// Global variable to hold the current setpoint (in encoder ticks)
float currentSetpointTicks = 0;

// Variables for freeze logic
boolean graphFrozen = false;
float freezeStartTime = -1;  // Timestamp when error condition started

// Define the window (in seconds) for the scrolling graph.
float graphWindow = 10;

// Conversion factor from ticks to degrees (for a 1600 ticks per 360° encoder)
final float TICKS_TO_DEGREES = 360.0 / 1600.0;

void setup() {
  size(950, 600);  // Increase width for more room

  cp5 = new ControlP5(this);
  graphPoints = new ArrayList<PVector>();
  
  // Layout variables for the left control panel
  int left = 20;
  int top = 20;
  int spacing = 40;
  int ctrlWidth = 180;
  int ctrlHeight = 20;
  
  // Setup Serial Port Dropdown
  portList = cp5.addDropdownList("Serial Ports")
                .setPosition(left, top)
                .setSize(ctrlWidth, 100);
  String[] ports = Serial.list();
  for (int i = 0; i < ports.length; i++) {
    portList.addItem(ports[i], i);
  }
  
  // Create PID Toggle button (initially locked)
  cp5.addToggle("PID_Toggle")
     .setPosition(left, top + spacing * 3)
     .setSize(50, ctrlHeight)
     .setValue(false)
     .setLabel("PID");
  
  // Create Setpoint slider (0 to 360°)
  cp5.addSlider("setpoint")
     .setPosition(left, top + spacing * 4)
     .setSize(ctrlWidth, ctrlHeight)
     .setRange(0, 360)
     .setValue(0)
     .setLabel("Setpoint (°)");
     
  // Create Kp slider
  cp5.addSlider("Kp")
     .setPosition(left, top + spacing * 5)
     .setSize(ctrlWidth, ctrlHeight)
     .setRange(0, 10)
     .setValue(2.0)
     .setLabel("Kp");
     
  // Create Ki slider
  cp5.addSlider("Ki")
     .setPosition(left, top + spacing * 6)
     .setSize(ctrlWidth, ctrlHeight)
     .setRange(0, 1)
     .setValue(0.1)
     .setLabel("Ki");
     
  // Create Kd slider
  cp5.addSlider("Kd")
     .setPosition(left, top + spacing * 7)
     .setSize(ctrlWidth, ctrlHeight)
     .setRange(0, 10)
     .setValue(0.5)
     .setLabel("Kd");
     
  // Create KI Threshold slider (0 to 100 ticks)
  cp5.addSlider("KIThresh")
     .setPosition(left, top + spacing * 8)
     .setSize(ctrlWidth, ctrlHeight)
     .setRange(0, 100)
     .setValue(20)
     .setLabel("KI Threshold");
     
  // Create Reset Encoder button
  cp5.addButton("Reset_Encoder")
     .setPosition(left, top + spacing * 9 + 10)
     .setSize(ctrlWidth, 30)
     .setLabel("Reset Encoder");
  
  // Lock all controllers until a serial port is chosen.
  lockControllers(true);
}

void draw() {
  background(200);
  
  // Define the graph drawing area (shifted right to avoid overlap with left panel)
  int graphX = 300;
  int graphY = 10;
  int graphW = width - graphX - 20;
  int graphH = height - 20;
  
  // Draw graph background and border
  fill(255);
  rect(graphX, graphY, graphW, graphH);
  stroke(0);
  noFill();
  rect(graphX, graphY, graphW, graphH);
  
  // Clone graphPoints safely
  ArrayList<PVector> safePoints;
  synchronized(graphPoints) {
    safePoints = new ArrayList<PVector>(graphPoints);
  }
  
  // Auto-scale vertical axis (encoder ticks) with ±20 tick buffer.
  float yMin, yMax;
  if (safePoints.size() > 0) {
    yMin = safePoints.get(0).y;
    yMax = safePoints.get(0).y;
    for (PVector pt : safePoints) {
      if (pt.y < yMin) yMin = pt.y;
      if (pt.y > yMax) yMax = pt.y;
    }
    yMin -= 20;
    yMax += 20;
    if (yMin == yMax) {
      yMin -= 20;
      yMax += 20;
    }
  } else {
    yMin = -1600;
    yMax = 1600;
  }
  
  // Define x-axis scrolling window
  float currentWindowEnd = (safePoints.size() > 0) ? safePoints.get(safePoints.size()-1).x : graphWindow;
  float currentWindowStart = currentWindowEnd - graphWindow;
  
  // Draw blue horizontal line at the current setpoint (in ticks)
  float setpointY = map(currentSetpointTicks, yMin, yMax, graphH + graphY, graphY);
  stroke(0, 0, 255);
  line(graphX, setpointY, graphX + graphW, setpointY);
  
  // Draw graph data if available
  if (safePoints.size() > 1) {
    stroke(255, 0, 0);
    noFill();
    beginShape();
    for (PVector pt : safePoints) {
      float x = map(pt.x, currentWindowStart, currentWindowEnd, graphX, graphX + graphW);
      float y = map(pt.y, yMin, yMax, graphH + graphY, graphY);
      vertex(x, y);
    }
    endShape();
  }
  
  // Draw Y-axis scale in degrees (converted from ticks)
  stroke(0);
  fill(0);
  textSize(10);
  textAlign(RIGHT, CENTER);
  int numYTicks = 4;  // Fewer tick marks for better spacing
  float yTickInterval = (yMax - yMin) / numYTicks;
  for (int i = 0; i <= numYTicks; i++) {
    float tickValue = yMin + i * yTickInterval;  // in ticks
    float tickDegrees = tickValue * TICKS_TO_DEGREES; // convert to degrees
    float yPos = map(tickValue, yMin, yMax, graphH + graphY, graphY);
    line(graphX - 5, yPos, graphX, yPos);
    text(nf(tickDegrees, 1, 1) + "°", graphX - 10, yPos);
  }
  
  // Draw X-axis scale (time in seconds)
  textAlign(CENTER, TOP);
  int numXTicks = 5;
  float xTickInterval = graphWindow / numXTicks;
  for (int i = 0; i <= numXTicks; i++) {
    float tickTime = currentWindowStart + i * xTickInterval;
    float xPos = map(tickTime, currentWindowStart, currentWindowEnd, graphX, graphX + graphW);
    line(xPos, graphY + graphH, xPos, graphY + graphH + 5);
    text(nf(tickTime, 1, 1) + "s", xPos, graphY + graphH + 10);
  }
  
  // Display last received serial message for debugging
  textAlign(LEFT, BASELINE);
  fill(0);
  text("Received: " + receivedLine, 10, height - 10);
}

////////////////////
// Helper: Lock or unlock GUI controls
void lockControllers(boolean lock) {
  cp5.getController("PID_Toggle").setLock(lock);
  cp5.getController("setpoint").setLock(lock);
  cp5.getController("Kp").setLock(lock);
  cp5.getController("Ki").setLock(lock);
  cp5.getController("Kd").setLock(lock);
  cp5.getController("KIThresh").setLock(lock);
  cp5.getController("Reset_Encoder").setLock(lock);
}

////////////////////
// Helper: Send default settings to Arduino on first connection.
void sendDefaultSettings() {
  if (myPort != null) {
    myPort.write("PID:OFF\n");
    myPort.write("Kp:" + cp5.getController("Kp").getValue() + "\n");
    myPort.write("Ki:" + cp5.getController("Ki").getValue() + "\n");
    myPort.write("Kd:" + cp5.getController("Kd").getValue() + "\n");
    myPort.write("KIThresh:" + int(cp5.getController("KIThresh").getValue()) + "\n");
    myPort.write("SP:" + cp5.getController("setpoint").getValue() + "\n");
  }
}

////////////////////
// GUI Control Callbacks

// Called when a serial port is selected from the dropdown.
void controlEvent(ControlEvent theEvent) {
  if (theEvent.isFrom("Serial Ports")) {
    int index = int(theEvent.getValue());
    String portName = Serial.list()[index];
    if (myPort != null) {
      myPort.stop();
    }
    myPort = new Serial(this, portName, 115200);
    myPort.bufferUntil('\n');
    lockControllers(false);
    sendDefaultSettings();
  }
}

// Toggle PID on/off; send parameters when enabling, clear/reset graph and freeze variables.
void PID_Toggle(boolean theValue) {
  pidEnabled = theValue;
  if (pidEnabled) {
    cp5.get(Toggle.class, "PID_Toggle").setLabel("PID ON");
    if (myPort != null) {
      myPort.write("PID:ON\n");
      myPort.write("Kp:" + cp5.getController("Kp").getValue() + "\n");
      myPort.write("Ki:" + cp5.getController("Ki").getValue() + "\n");
      myPort.write("Kd:" + cp5.getController("Kd").getValue() + "\n");
      myPort.write("KIThresh:" + int(cp5.getController("KIThresh").getValue()) + "\n");
      myPort.write("SP:" + cp5.getController("setpoint").getValue() + "\n");
    }
    baselineTime = -1;
    synchronized(graphPoints) {
      graphPoints.clear();
    }
    graphFrozen = false;
    freezeStartTime = -1;
  } else {
    cp5.get(Toggle.class, "PID_Toggle").setLabel("PID OFF");
    if (myPort != null) {
      myPort.write("PID:OFF\n");
    }
    synchronized(graphPoints) {
      graphPoints.clear();
    }
    graphFrozen = false;
    freezeStartTime = -1;
  }
}

// Setpoint slider callback: send new setpoint, unfreeze and clear the graph.
void setpoint(float theValue) {
  if (myPort != null) {
    myPort.write("SP:" + theValue + "\n");
    graphFrozen = false;
    freezeStartTime = -1;
    synchronized(graphPoints) {
      graphPoints.clear();
    }
  }
}

// PID gain callbacks
void Kp(float theValue) {
  if (myPort != null) {
    myPort.write("Kp:" + theValue + "\n");
  }
}

void Ki(float theValue) {
  if (myPort != null) {
    myPort.write("Ki:" + theValue + "\n");
  }
}

void Kd(float theValue) {
  if (myPort != null) {
    myPort.write("Kd:" + theValue + "\n");
  }
}

void KIThresh(float theValue) {
  if (myPort != null) {
    myPort.write("KIThresh:" + int(theValue) + "\n");
  }
}

// Reset Encoder button callback
void Reset_Encoder() {
  if (myPort != null) {
    myPort.write("ResetEnc\n");
  }
  synchronized(graphPoints) {
    graphPoints.clear();
  }
  baselineTime = -1;
  graphFrozen = false;
  freezeStartTime = -1;
}

////////////////////
// Serial Event: Process incoming data from Arduino.
void serialEvent(Serial p) {
  String inString = p.readStringUntil('\n');
  if (inString != null) {
    inString = inString.trim();
    receivedLine = inString;
    // Expected format: "DATA,<timestamp>,<encoderTicks>,<targetTicks>"
    if (inString.startsWith("DATA,")) {
      String[] parts = split(inString, ',');
      if (parts.length >= 4) {
        float timestamp = float(parts[1]);
        float encoderVal = float(parts[2]);
        currentSetpointTicks = float(parts[3]);  // update current setpoint (in ticks)
        if (pidEnabled && !graphFrozen) {
          if (abs(encoderVal - currentSetpointTicks) < 5) {
            if (freezeStartTime < 0) {
              freezeStartTime = timestamp;
            } else if (timestamp - freezeStartTime >= 2000) {
              graphFrozen = true;
            }
          } else {
            freezeStartTime = -1;
          }
          if (!graphFrozen) {
            if (baselineTime < 0) {
              baselineTime = timestamp;
            }
            float relTime = (timestamp - baselineTime) / 1000.0;
            synchronized(graphPoints) {
              graphPoints.add(new PVector(relTime, encoderVal));
              while (graphPoints.size() > 0 && graphPoints.get(0).x < relTime - graphWindow) {
                graphPoints.remove(0);
              }
            }
          }
        }
      }
    }
  }
}
