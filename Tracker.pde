// by Les Hall
// started Sun Nov 9 2014
// from oscP5 examples
// added head Tracking onWed Dec 24 2014


import oscP5.*;
import netP5.*;
import java.awt.*;
import java.awt.event.InputEvent;
import processing.video.*;
import blobscanner.*;


OscP5 oscP5;
NetAddress myRemoteLocation;
java.awt.Robot robo;
Capture cam;
Detector bd;

float responseTime = 2.0;
float mouseRate = 8.0;
PVector blob = new PVector(0, 0, 0);
PVector blobAvg = new PVector(0, 0, 0);
PVector gyro = new PVector(0, 0, 0);
PVector accel = new PVector(0, 0, 0);
PVector vel = new PVector(0, 0, 0);
PVector dist = new PVector(0, 0, 0);
int numButtons = 9;
boolean[] button = new boolean[9];
PVector mousePos = new PVector(0, 0, 0);
PImage img;
PVector pos = new PVector(0, 0, 0);
PVector ref = new PVector(0, 0, 0);
float threshold = 0.5;
int blur = 2;
boolean inverse = false;
float f = 25;
PVector[] UIlocations = {
  new PVector(0, 0*f, 0), 
  new PVector(0, 1*f, 0), 
  new PVector(0, 2*f, 0), 
  new PVector(0, 3*f, 0), 
  new PVector(0, 4*f, 0), 
  new PVector(0, 5*f, 0), 
  new PVector(0, 6*f, 0)};
int UIpointer = 0;
float tau0 = 0.7;
float tau1 = 0.95;
boolean firstRun = true;
int size = 300;
int k = size/16;
PVector invert = new PVector(1, 1, 0);


void setup()
{
  size(3*size, size);
  frameRate(15);
    
  /* start oscP5, listening for incoming messages */
  oscP5 = new OscP5(this, 11000);
  myRemoteLocation = new NetAddress("127.0.0.1", 11000);

  // Robot class
  try
  { 
    robo = new java.awt.Robot();
  } 
  catch (AWTException e)
  {
    e.printStackTrace();
  }
  
  String[] cameras = Capture.list();

  if (cameras == null)
  {
    println("Failed to retrieve the list of available cameras, will try the default...");
    exit();
    //cam = new Capture(this, width, height);
  }
  if (cameras.length == 0)
  {
    println("There are no cameras available for capture.");
    exit();
  }
  else
  {
    println("Available cameras:");
    for (int i = 0; i < cameras.length; i++)
    {
      println(cameras[i]);
    }

    // The camera can be initialized directly using an element
    // from the array returned by list():
    cam = new Capture(this, cameras[3]);
    // Or, the settings can be defined based on the text in the list
    //cam = new Capture(this, 640, 480, "Built-in iSight", 30);
    
    // Start capturing the images from the camera
    cam.start();
  }
  
  img = new PImage(size, size);  

  bd = new Detector(this, 0);
}


void draw()
{
  if (firstRun)
  {
    frame.setLocation(displayWidth/2 - width/2, displayHeight/2 - height/2);
    firstRun = false;
  }
  background(63, 0, 127); 
 
  // get a camera image
  if (cam.available() == true)
  {
    cam.read();
    img.copy(cam, 
      cam.width/2-img.width/2, cam.height/2-img.height/2, img.width, img.height, 
      0, 0, img.width, img.height);
      
    //bd.setThreshold( int(255*threshold) );
    img.filter(GRAY);
    img.filter(BLUR, blur);
    img.filter(THRESHOLD,  threshold);

    if (inverse)
      for (int i=0; i<img.pixels.length; ++i)
        img.pixels[i] = color(255, 255, 255) - img.pixels[i];      
    
    img.loadPixels();

    // plot image
    pushMatrix();
      translate(size, 0);
      scale(-1.0, 1.0);
      image(img, -img.width, 0);
    popMatrix();
    
    drawUserInterface();
    
    // scan for blobs
    bd.imageFindBlobs(img);  // find the blobs
    bd.loadBlobsFeatures();  // get blob features
    bd.findCentroids();  // get center of blobs
    bd.weightBlobs(true);  /// get size of blobs

    // calculate average head motion
    int numBlobs = bd.getBlobsNumber();
    if (numBlobs > 0)
    {
      float x = 0;
      float y = 0;
      int counter = 0;
      for (int i=0; i<numBlobs; ++i)
      {
        // get blob features
        float bdx = bd.getCentroidX(i);
        float bdy = bd.getCentroidY(i);
        
        if ( (bdx >= 0) && (bdx < img.width) && 
          (bdy >=0) && (bdy < img.height) )
        {
          // make calculations
          x += bdx;
          y += bdy;
          ++counter;
          
          // plot blob centroids    
          pushMatrix();
            fill(0, 255, 0, 63);
            ellipse(size + img.width-1 - bdx, bdy, k, k);
          popMatrix();
        }
        
      } 
      if (counter > 0)
      {
        x /= counter;
        y /= counter;
    
        // keep running averages of position and reference
        pos.x = tau0*pos.x + (1.0-tau0)*x;
        pos.y = tau0*pos.y + (1.0-tau0)*y;
        ref.x = tau1*ref.x + (1.0-tau1)*pos.x;
        ref.y = tau1*ref.y + (1.0-tau1)*pos.y;
      }
    }

    // plot dots
    //
    pushMatrix();
      // 
      // plot long running average
      fill(255, 0, 255);
      ellipse(size + img.width-1 - ref.x, ref.y, k, k);
      //
      // plot short running average
      fill(0, 255, 255);
      ellipse(size + img.width-1 - pos.x, pos.y, k, k);
    popMatrix();
  }
  
  // adjust mouse position
  mousePos.x += invert.x * (-mouseRate * gyro.y - (pos.x - ref.x) );
  mousePos.y += invert.y * (-mouseRate * gyro.x + (pos.y - ref.y) );
  if (mousePos.x < 0) mousePos.x = 0;
  if (mousePos.x >= (displayWidth - 1) ) mousePos.x = displayWidth - 1;
  if (mousePos.y < 0) mousePos.y = 0;
  if (mousePos.y >= (displayHeight - 1) ) mousePos.y = displayHeight - 1;
  
  // send Robot class command to move the mouse!
  robo.mouseMove( int(mousePos.x), int(mousePos.y) );

  // mouse buttons
  if (button[0] == true)
  {
    button[0] = false;
    robo.mousePress(InputEvent.BUTTON1_MASK);
    robo.mouseRelease(InputEvent.BUTTON1_MASK);
  }
  if (button[6] == true)
  {
    button[6] = false;
    robo.mousePress(InputEvent.BUTTON2_MASK);
    robo.mouseRelease(InputEvent.BUTTON2_MASK);
  }
  if (button[3] == true)
  {
    button[3] = false;
    robo.mousePress(InputEvent.BUTTON3_MASK);
    robo.mouseRelease(InputEvent.BUTTON3_MASK);
  }
}




void keyPressed()
{
  if (key == CODED)
  {
    if (keyCode == UP)
    {
      --UIpointer;
      if (UIpointer < 0)
        UIpointer = UIlocations.length - 1;
    }
    else if (keyCode == DOWN)
    {
      ++UIpointer;
      UIpointer %= UIlocations.length;
    }
    else if (UIpointer == 0)
    {
      if (keyCode == RIGHT)
      {
        threshold += 0.05;
        threshold = round(20 * threshold) / 20.0;
        if (threshold > 1)
          threshold = 1;
      }
      if (keyCode == LEFT)
      {
        threshold -= 0.05;
        threshold = round(20 * threshold) / 20.0;
        if (threshold < 0)
          threshold = 0;
      }
    }
    else if (UIpointer == 1)
    {
      if (keyCode == RIGHT)
      {
        blur += 1;
        if (blur > 16)
          blur = 16;
      }
      if (keyCode == LEFT)
      {
        blur -= 1;
        if (blur < 0)
          blur = 0;
      }
    }
    else if (UIpointer == 2)
    {
      if (keyCode == RIGHT)
      {
        inverse = true;
      }
      if (keyCode == LEFT)
      {
        inverse = false;
      }
    }
    else if (UIpointer == 3)
    {
      if (keyCode == RIGHT)
      {
        tau0 += 0.05;
        tau0 = round(20 * tau0) / 20.0;
        if (tau0 > 1)
          tau0 = 1;
      }
      if (keyCode == LEFT)
      {
        tau0 -= 0.05;
        tau0 = round(20 * tau0) / 20.0;
        if (tau0 < 0)
          tau0 = 0;
      }
    }
    else if (UIpointer == 4)
    {
      if (keyCode == RIGHT)
      {
        tau1 += 0.05;
        tau1 = round(20 * tau1) / 20.0;
        if (tau1 > 1)
          tau1 = 1;
      }
      if (keyCode == LEFT)
      {
        tau1 -= 0.05;
        tau1 = round(20 * tau1) / 20.0;
        if (tau1 < 0)
          tau1 = 0;
      }
    }
    else if (UIpointer == 5)
    {
      if (keyCode == RIGHT)
      {
        invert.x = 1;
      }
      if (keyCode == LEFT)
      {
        invert.x = -1;
      }
    }
    else if (UIpointer == 6)
    {
      if (keyCode == RIGHT)
      {
        invert.y = 1;
      }
      if (keyCode == LEFT)
      {
        invert.y = -1;
      }
    }
  }
  else
  {
  }
}




void drawUserInterface()
{
  // draw parameters
  textSize(f);
  pushMatrix();
    translate(2*size, 0);  
    textAlign(LEFT, TOP);
    fill(0, 255, 0);
    if (UIpointer == 0) fill(255, 0, 0);
    text("threshold = " + str(threshold), 
      UIlocations[0].x, UIlocations[0].y);
    fill(0, 255, 0);
    if (UIpointer == 1) fill(255, 0, 0);
    text("blur = " + str(blur), 
      UIlocations[1].x, UIlocations[1].y);
    fill(0, 255, 0);
    if (UIpointer == 2) fill(255, 0, 0);
    text("inverse = " + str(inverse), 
      UIlocations[2].x, UIlocations[2].y);
    fill(0, 255, 0);
    if (UIpointer == 3) fill(255, 0, 0);
    text("tau0 = " + str(tau0), 
      UIlocations[3].x, UIlocations[3].y);
    fill(0, 255, 0);
    if (UIpointer == 4) fill(255, 0, 0);
    text("tau1 = " + str(tau1), 
      UIlocations[4].x, UIlocations[4].y);
    fill(0, 255, 0);
    if (UIpointer == 5) fill(255, 0, 0);
    text("invert x = " + str(invert.x), 
      UIlocations[5].x, UIlocations[5].y);
    fill(0, 255, 0);
    if (UIpointer == 6) fill(255, 0, 0);
    text("invert y = " + str(invert.y), 
      UIlocations[6].x, UIlocations[6].y);
  popMatrix();

  // draw instructions  
  pushMatrix();
    translate(0, 0); 
    textAlign(LEFT, TOP);
    fill(255, 255, 0);
    text("use arrows keys to", 
      0, 0);
    text("select and change", 
      0, f);
    text("red values", 
      0, 2*f);
  popMatrix();
}



float sigmoid(float x)
{
  return 2.0/(1.0 + exp(-x)) - 1;
}




/* incoming osc message are forwarded to the oscEvent method. */
void oscEvent(OscMessage theOscMessage)
{
  // grab the gyro data
  String g = "/gyrosc/gyro";
  String a = "/gyrosc/accel";
  String b = "/gyrosc/button";
  if(g.equals(theOscMessage.addrPattern() ) )
  {
    gyro.x = theOscMessage.get(0).floatValue();  // pitch
    gyro.z = theOscMessage.get(1).floatValue();  // roll
    gyro.y = theOscMessage.get(2).floatValue();  // yaw
  }
  else if(a.equals(theOscMessage.addrPattern() ) )
  {
    accel.x = theOscMessage.get(0).floatValue();  // x axis
    accel.y = theOscMessage.get(1).floatValue();  // y axis
    accel.z = theOscMessage.get(2).floatValue();  // z axis
  }
  else if(b.equals(theOscMessage.addrPattern() ) )
  {
    button[theOscMessage.get(0).intValue()-1] = true;
  }
}

