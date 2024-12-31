package frc.robot.subsystems.objectdetection;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Base64;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfByte;
import org.opencv.imgcodecs.Imgcodecs;
// import org.opencv.core.MatOfByte;
// import org.opencv.imgcodecs.Imgcodecs;
import org.photonvision.simulation.PhotonCameraSim;

import java.io.BufferedReader;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import org.eclipse.jetty.util.IO;
import org.kobe.xbot.Client.XTablesClient;;
public class LocalhostCommunicator {
    private Socket[] sockets;
    private CvSink[] camSinks;
    private ExecutorService executorService;
    private final String recieveKey = "Target_Waypoints";
    public LocalhostCommunicator(PhotonCameraSim[] cameraSims){
        int numberOfCameras = cameraSims.length;
        this.sockets = new Socket[numberOfCameras];
        this.camSinks = new CvSink[numberOfCameras];
        try{
            int socketStart = 5000;
            for(int i = 0;i<numberOfCameras;i++){
                sockets[i] = new Socket("localhost",i+socketStart);
                camSinks[i] = CameraServer.getVideo(cameraSims[i].getCamera().getName() + "-raw");
            }
        }
        catch(Exception e){
            DriverStation.reportError("Failed to create sockets\n" + e.getMessage(), e.getStackTrace());
        }
        this.executorService = Executors.newFixedThreadPool(numberOfCameras);
        

    }
    
    private void sendFrame(Mat frame,Socket socket) {
            // Encode Mat to JPEG format
        MatOfByte buffer = new MatOfByte();
        Imgcodecs.imencode(".jpg", frame, buffer);
        byte[] byteArray = buffer.toArray();
        try {
            OutputStream outputStream = socket.getOutputStream();
            outputStream.write(byteArray);
            outputStream.flush();
        } catch (Exception e) {
            DriverStation.reportError("Error sending over socket!\n" + e.getMessage(), e.getStackTrace());
        }
    }
    
    // private String readPythonResponse(Socket socket){
    //     String response = xTablesClient.getString(recieveKey).complete();
    //     do stuff
    //     return response;
    // }

    public void updateAllCameraFrames(){
        for(int i = 0;i<camSinks.length;i++){
            int finalI = i;
            executorService.execute(() ->{
                Mat frame = new Mat();
                camSinks[finalI].grabFrame(frame);
                sendFrame(frame, sockets[finalI]);
            });
        }

    }

    public void close(){
        for(int i = 0;i<camSinks.length;i++){
            try{
                sockets[i].close();
                camSinks[i].close();
            }
            catch(IOException e){
                DriverStation.reportError("Error closing materials!\n" + e.getMessage(), e.getStackTrace());
            }
        }
        executorService.shutdownNow();
    }

   
    

}

 