package frc.robot;

import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Timer;

public class TapeThread extends Thread {

Timer timer;
CvSink sink;
TapePipe pipe;
Mat mat;
Rect r;

int x;


 public void run(){
    x = 0;
     timer = new Timer();
    sink = Robot.getCameraServer().getVideo();
     pipe = new TapePipe();
     mat = new Mat();
     r = new Rect();

     while(true){
        x = centerVisionRect();
     }

 }

 public int centerVisionRect(){
    timer.start();
    sink.grabFrame(mat);
    if (mat.width() < 1){
        return 0;
    }
    pipe.process(mat);
    mat = pipe.blurOutput();
    Rect r = Imgproc.boundingRect(mat);
    timer.stop();
    timer.reset();
    if(r.width >= 20){
    return (r.x + (r.width/2) - 400);
    } else {
    return 0;
    }
  }
}