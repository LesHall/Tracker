// by Les Hall
// started Sun Nov 9 2014
// from oscP5 examples
// 


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
float avgx = 0;
float avgy = 0;
float prevx = 0;
float prevy = 0;
float q = 8;  // size of window


void setup()
{
  size(256, 256);
  //frameRate(15);
    
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
  
  img = new PImage(width, height);  

  bd = new Detector(this);
}


void draw()
{
  background(0); 
 
  // get a camera image
  if (cam.available() == true)
  {
    cam.read();
    img.copy(cam, 
      cam.width/2-width/2, cam.height/2-height/2, width, height, 
      0, 0, width, height);
    img.filter(GRAY);
    img.filter(THRESHOLD, 0.25);


    pushMatrix();
      scale(-1.0, 1.0);
      image(img, -img.width, 0);
    popMatrix();
    
    img.loadPixels();
    
    // scan for blobs
    bd.imageFindBlobs(img);  // find the blobs
    bd.loadBlobsFeatures();  // get blob features
    bd.findCentroids();  // get center of blobs
    bd.weightBlobs(true);  /// get size of blobs

    // calculate average head motion
    int numBlobs = bd.getBlobsNumber();
    float prevtau = 1.0/frameCount;
    prevx = prevtau*prevx + (1.0-prevtau)*avgx;
    prevy = prevtau*prevy + (1.0-prevtau)*avgy;
    prevx = 0;
    prevy = 0;
    float x = 0;
    float y = 0;
    int k = height/32;
    for (int i=0; i<numBlobs; ++i)
    {
      float bdx = bd.getCentroidX(i);
      float bdy = bd.getCentroidY(i);
      float dx = (bdx - width/2);
      float dy = (bdy - height/2);
      x += dx;
      y += dy;
      // plot blob centroids    
      fill(0, 255, 0);
      ellipse(width-1 - bd.getCentroidX(i), bd.getCentroidY(i), k, k);
    } 
    x /= numBlobs;
    y /= numBlobs;
    float tau = 0.0;
    avgx = tau*avgx + (1.0-tau)*x;
    avgy = tau*avgy + (1.0-tau)*y;

    // plot dots
    //
    // plot long running average
    fill(255, 0, 255);
    ellipse(width/2 - prevx, height/2 + prevy, 2*k, 2*k);
    //
    // plot short running average
    fill(0, 255, 255);
    ellipse(width/2 - avgx, height/2 + avgy, 2*k, 2*k);
    //
    // plot mouse indicator in yellow
    float wx = float(width)/float(displayWidth);
    float wy = float(height)/float(displayHeight);
    fill(255, 255, 0);
    ellipse(wx*mousePos.x, wy*mousePos.y, 2*k, 2*k);
  }
  
  // adjust mouse position
  float threshold = 1;
  float s = displayWidth/64;
  mousePos.x -= mouseRate * gyro.y + pow( (avgx-prevx) / responseTime / frameRate, 3);
  mousePos.y -= mouseRate * gyro.x - pow( (avgy-prevy) * 2 / responseTime / frameRate, 3);
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

