
// Graphing sketch


// This program takes ASCII-encoded strings
// from the serial port at 9600 baud and graphs them. It expects values in the
// range 0 to 1023, followed by a newline, or newline and carriage return

// Created 20 Apr 2005
// Updated 18 Jan 2008
// by Tom Igoe
// This example code is in the public domain.

import processing.serial.*;

Serial myPort;        // The serial port
int graphTop = 323;         // top horizontal position of the graph
int xPosLeft = 124;
int xPosRight = 350;

void setup () {
  // set the window size:
  size(600, 400);        

  // List all the available serial ports
  println(Serial.list());
  // I know that the first port in the serial list on my mac
  // is always my  Arduino, so I open Serial.list()[0].
  // Open whatever port is the one you're using.
  myPort = new Serial(this, Serial.list()[1], 115200);
  // don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');
  // set inital background:
  PImage img;
  img = loadImage("background.png");
  background(img);
}
void draw () {
  // everything happens in the serialEvent()
}

void serialEvent (Serial myPort) {
  // get the ASCII string:
  String inString = myPort.readStringUntil('\n');

  if (inString != null) {
    // trim off any whitespace:
    inString = trim(inString);
    // split the string on the commas and convert the
    // resulting substrings into an integer array:
    float[] bands = float(split(inString, ","));

    // if the array has at least four-teen elements, you know
    // you got the whole thing.  Put the numbers in the
    // color variables:
    if (bands.length >=14) {
      for ( int i=0; i < 14; i++) {
        // convert to an int and map to the screen height:
        bands[i] = map(bands[i], 0, 1023, 0, 274);
      }
      for (int i=0; i < 7; i++) { 
        // draw the line:
        stroke(127, 34, 255);
        strokeCap(SQUARE);
        strokeWeight(28);
        line(xPosLeft, graphTop, xPosLeft, graphTop - bands[i]);
        stroke(0, 0, 0);
        line(xPosLeft, 49, xPosLeft, graphTop - bands[i]);
        if (i < 6) {
          xPosLeft += 29;
        }
        else {
          xPosLeft = 124;
        }
      }
      for (int i=7; i < 14; i++) {
        // draw the line:
        stroke(127, 34, 255);
        strokeCap(SQUARE);
        strokeWeight(28);
        line(xPosRight, graphTop, xPosRight, graphTop - bands[i]);
        stroke(0, 0, 0);
        // strokeWeight(5);
        line(xPosRight, 49, xPosRight, graphTop - bands[i]);  
        if (i < 13) {
          xPosRight += 29;
        }
        else {
          xPosRight = 350;
        }
      }
    }
  }
}