import processing.serial.*;
String xLabel = "X-axis";
String yLabel = "Y-axis";
String Heading = "Test Plot";
Serial myPort;        // The serial port
int xPos = 1;         // horizontal position of the graph
float inByte = 0;
float inByte1=0,inByte2=0;
int input=0;
PFont font;
int text_offset=20; 
int margin_offset=100;
 
void setup () 
{
 
// set the window size: 100 points taken for Margins
  size(2500,1500);
 
// List all the available serial ports - For reference
  println(Serial.list());
 
// Select first port
  myPort = new Serial(this, Serial.list()[0], 115200);
 
// Don't generate a serialEvent() unless you get a newline character:
  myPort.bufferUntil('\n');
 
// set inital background:
  background(255);
 
  noLoop();  
}
 
void draw () 
{
 
      // Copy the old data to inByte2 & copy new data to inByte1
      inByte2=inByte1;
      inByte1=inByte;
 
      // Draw a line between inByte1 & inByte2
      line(xPos+margin_offset-1, height-inByte2-margin_offset, xPos+margin_offset, height - inByte1-margin_offset);  
 
     // Reset if the edge of the screen is reached
      if (xPos >= width) 
     {
 
        //Reset xPos 
        xPos = 0;
 
        //Reset background 
        background(255);
      } 
 
      else
      {
          // increment the horizontal position:
          xPos++;
      }
}
 
 
void serialEvent (Serial myPort) 
{
   // get the ASCII string:
   String inString = myPort.readStringUntil('\n');
 
   if (inString != null) 
   {
       inByte = float(inString);
       redraw();
   }
 }
