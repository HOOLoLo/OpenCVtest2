package com.example.administrator.opencvtest2;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.StringTokenizer;
import java.util.Vector;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewFrame;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.CameraBridgeViewBase.CvCameraViewListener2;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import android.app.Activity;
import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Bundle;
import android.util.Log;
import android.view.Menu;
import android.view.MenuItem;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.TextView;

import static org.opencv.core.CvType.CV_32F;
import static org.opencv.core.CvType.CV_32FC1;
import static org.opencv.core.CvType.CV_8UC1;
import static org.opencv.core.CvType.CV_8UC3;
import static org.opencv.core.CvType.CV_8UC4;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_NONE;
import static org.opencv.imgproc.Imgproc.CHAIN_APPROX_SIMPLE;
import static org.opencv.imgproc.Imgproc.COLOR_RGB2HSV;
import static org.opencv.imgproc.Imgproc.CV_WARP_FILL_OUTLIERS;
import static org.opencv.imgproc.Imgproc.MORPH_CLOSE;
import static org.opencv.imgproc.Imgproc.MORPH_OPEN;
import static org.opencv.imgproc.Imgproc.MORPH_RECT;
import static org.opencv.imgproc.Imgproc.RETR_EXTERNAL;
import static org.opencv.imgproc.Imgproc.RETR_TREE;
import static org.opencv.imgproc.Imgproc.cvtColor;
import static org.opencv.imgproc.Imgproc.getStructuringElement;
import static org.opencv.imgproc.Imgproc.initUndistortRectifyMap;
import static org.opencv.imgproc.Imgproc.morphologyEx;
import static org.opencv.imgproc.Imgproc.warpAffine;

public class ImageManipulationsActivity extends Activity implements CvCameraViewListener2,SensorEventListener {
    private String TAG = "OpenCV_Test";
    //OpenCV的相机接口
    private CameraBridgeViewBase mCVCamera;
    //按钮组件
    private Button mButton;
    //当前处理状态
    private static int Cur_State = 0;
    //缓存相机每帧输入的数据
    private Mat mRgba;
    private Mat hsv_h;
    private Mat hsv_s;
    private Mat hsv_v;

    private TextView tv_H_data;
    private TextView tv_S_data;
    private TextView tv_V_data;

    private int H_data;
    private int S_data;
    private int V_data;

    private SensorManager sensorManager;
    private Sensor gSensor;
    private Sensor mSensor;
    private float[] mGData = new float[3];
    private float[] mMData = new float[3];
    private float[] mR = new float[16];
    private float[] mI = new float[16];
    private float[] mOrientation = new float[3];
    private double[][]map=new double[2][3];
    private TextView tv_Yaw;
    private TextView tv_Roll;
    private TextView tv_Pitch;
    private TextView tv_location;
    private int core_width;
    private int core_height;
    /**
     * 通过OpenCV管理Android服务，异步初始化OpenCV
     */
    private BaseLoaderCallback mLoaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status){
            switch (status) {
                case LoaderCallbackInterface.SUCCESS:
                    Log.i(TAG,"OpenCV loaded successfully");
                    mCVCamera.enableView();
                    mCVCamera.setMaxFrameSize(780,540);

                    break;
                default:
                    break;
            }
        }
    };

//    //不均匀光照补偿
//    public void lightCompensate(Mat image,int blocksize,int rows,int cols){
//        double new_rows=Math.ceil(((double) rows)/(double) blocksize);
//        double new_cols=Math.ceil((double)cols/(double)blocksize);
//        Mat blockImage=new Mat((int) new_rows, ((int) new_cols),CV_32FC1);
//        for(int i=0;i<new_rows;i++){
//            for(int j=0;j<new_rows;j++){
//                int rowmin=i*blocksize;
//                int rowmax=(i+1)*blocksize;
//                if(rowmax>rows)rowmax=rows;//没有整除的情况下最后一块会不够
//                int colmin=i*blocksize;
//                int colmax=(i+1)*blocksize;
//                if(colmax>cols)colmax=cols;
//
//                Mat imageROI=image.colRange(colmin,colmax).rowRange(rowmin,rowmax);
//                Scalar temaver=Core.mean(image);
//                double Temaver=temaver.val[0];
//                blockImage.put()
//            }
//        }
//
//    }
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.image_manipulations_surface_view);
        //tv_H_data =(TextView)findViewById(R.id.MAXH_data);
        //tv_S_data =(TextView)findViewById(R.id.MAXS_data);
        //tv_V_data =(TextView)findViewById(R.id.MAXV_data);

        sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        gSensor = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
        mSensor = sensorManager.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD);

        sensorManager.registerListener(this, gSensor,
                SensorManager.SENSOR_DELAY_GAME);
        sensorManager.registerListener(this, mSensor,
                sensorManager.SENSOR_DELAY_GAME);

        tv_Yaw=(TextView)findViewById(R.id.Yaw);
        tv_Pitch=(TextView)findViewById(R.id.Pitch);
        tv_Roll=(TextView)findViewById(R.id.roll);
        mCVCamera = (CameraBridgeViewBase) findViewById(R.id.camera_view);
        mCVCamera.setCvCameraViewListener(this);
        mButton = (Button) findViewById(R.id.deal_btn);
        tv_location=(TextView)findViewById(R.id.loaction);
        mButton.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View v) {
                if (Cur_State < 2) {
                    //切换状态
                    Cur_State++;
                } else {
                    //恢复初始状态
                    Cur_State = 0;
                }
            }
        });
    }

    @Override
    public void onResume(){
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            Log.d(TAG,"OpenCV library not found!");
        } else {
            Log.d(TAG, "OpenCV library found inside package. Using it!");
            mLoaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }
    @Override
    public void onDestroy() {
        super.onDestroy();
        if(mCVCamera!=null){
            mCVCamera.disableView();
        }
    }
    @Override
    public void onPause(){
        super.onPause();
        sensorManager.unregisterListener(this);
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

        mRgba=new Mat(height,width, CvType.CV_8UC4);
    }

    @Override
    public void onCameraViewStopped() {
        mRgba.release();
        hsv_h.release();
        hsv_s.release();
        hsv_v.release();
    }


    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {


        int rows = (int) inputFrame.rgba().height();
        int cols = (int) inputFrame.rgba().width();

        mRgba=new Mat(rows,cols,CV_8UC4);
//        map[0][0]=1;
//        map[1][1]=1;
//        map[0][2]=360*(Math.tan(mOrientation[1]));
//        map[1][2]=240*(Math.tan(mOrientation[2]));
        //Scalar maps=new Scalar(1,0,360*(Math.tan(mOrientation[1])));
        //Scalar map2=new Scalar(0,1,240*(Math.tan(mOrientation[2])));
        Mat map_matrix=new Mat(2,3,CV_32F);
        map_matrix.put(0,0,1);
        map_matrix.put(0,2,360*(Math.tan(mOrientation[1])));
        map_matrix.put(1,1,1);
        map_matrix.put(1,2,240*(Math.tan(mOrientation[2])));
        warpAffine(inputFrame.rgba(),mRgba,map_matrix,new Size(720,480),CV_WARP_FILL_OUTLIERS);
        switch (Cur_State){
            case 1:
                    //-----直接取颜色数据来识别----//
                   // Imgproc.cvtColor(inputFrame.gray(),mRgba,Imgproc.COLOR_GRAY2BGR,4);
                    Mat hsv=new Mat();
                    Imgproc.cvtColor(mRgba,hsv,Imgproc.COLOR_RGB2HSV);
                   // List<Mat> mv=new ArrayList<Mat>();
                   // Core.split(hsv,mv);
                   // Imgproc.equalizeHist(mv.get(2),mv.get(2));
                    //Core.merge(mv,hsv);
                   Mat imgThreshold=new Mat();
//                    hsv_h=new Mat(rows,cols,CvType.CV_8UC1);
//                    hsv_s=new Mat(rows,cols,CvType.CV_8UC1);
//                    hsv_v=new Mat(rows,cols,CvType.CV_8UC1);
//                    hsv_h=mv.get(0);
//                    hsv_s=mv.get(1);
//                    hsv_v=mv.get(2);
                Core.inRange(hsv, new Scalar(0,170,50),new Scalar(5,255,255),imgThreshold);
//                Mat element=getStructuringElement(MORPH_RECT,new Size(5,5));
//                morphologyEx(imgThreshold,imgThreshold,MORPH_OPEN,element);
//                morphologyEx(imgThreshold,imgThreshold,MORPH_CLOSE,element);
//                Mat edges = new Mat(rows, cols, CV_8UC3);
//                Mat mTmp=new Mat(rows,cols,CvType.CV_8UC3);
//                Size ksize = new Size(5, 5);
//                ArrayList found=new ArrayList();
//                Imgproc.Canny(mTmp,edges , 100, 200);
//                List<MatOfPoint> contours=new ArrayList<MatOfPoint>();
//                Mat hierarchy=new Mat();

                mRgba=imgThreshold;

               // 取画面中红色的坐标。
               int i,j=0;
                byte []a=new byte[5];
                int k=0;
                core_height=0;
                core_width=0;

//                Mat mask=new Mat(rows,1,CV_8UC1);
//                for(byte i=0;i<rows;i++){
//                    byte []value=new byte[1];
//                    value[0]=i;
//                    mask.put(i,1,value);
//                }


                for( i=0;i<rows;i++){
                    for (j=0;j<cols;j++){
                        mRgba.get(i,j,a);
                        //不知道为什么255取出来是-1
                        if (a[0]<0){
                            k++;
                            core_height+=i;
                            core_width+=j;
                        }
                    }
                }
                core_height=core_height/k;
                core_width=core_width/k;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        tv_location.setText("x:"+ String.valueOf(core_width)+"y:"+String.valueOf(core_height));
                    }
                });





//                    //----s（饱和度）处理---//
//                    Mat matS1=new Mat(rows,cols,CvType.CV_8UC1);
//                    Mat matS2=new Mat(rows,cols,CvType.CV_8UC1);
//                    Mat matS3=new Mat(rows,cols,CvType.CV_8UC1);
//                    Mat element_Se =Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,new Size(5,5));
//                    Mat element_Sd =Imgproc.getStructuringElement(Imgproc.MORPH_ELLIPSE,new Size(5,5));
//                    Imgproc.threshold(hsv_s,matS1,90,255,Imgproc.THRESH_BINARY);
//                    Imgproc.erode(matS1,matS2,element_Se);
//                    Imgproc.dilate(matS2,matS3,element_Sd);
//
//                    //----v(明度)处理----//
//                    Mat matV1=new Mat(rows,cols,CvType.CV_8UC1);
//                    Imgproc.threshold(hsv_v,matV1,248,255,Imgproc.THRESH_BINARY);//这里把明度超过248的都筛选出来的作用是什么？
//
//                    //-----叠加----//
//                    Mat matadd=new Mat(rows,cols,CvType.CV_8UC1);
//                    Core.add(matS3,matV1,matadd);
//
//                    Mat imgDst= matadd;
//                    List<MatOfPoint> contours=new ArrayList<MatOfPoint>();
//                    Mat hierarchy=new Mat();
//                    Imgproc.findContours(imgDst,contours,hierarchy,RETR_EXTERNAL,CHAIN_APPROX_NONE);
//                    int mContoursNum=contours.size();
//
//                    int RstNum=0;
//                    for(int index=0;index<mContoursNum;index++){
//                        Size nPerSum=contours.get(index).size();
//
//                    }
//
//                    //----统计颜色---//
//                    int colWidth=()
//

                    //Imgproc.cvtColor(hsv_s,mRgba,Imgproc.COLOR_GRAY2RGB);

                /**
                //Canny边缘检测
                Mat mTmp=new Mat(rows,cols,CvType.CV_8UC3);
                Mat edges = new Mat(rows, cols, CV_8UC3);
               // mRgba = inputFrame.rgba();
                Size ksize = new Size(5, 5);
                Imgproc.GaussianBlur(mRgba, mTmp, ksize, 0, 0);
                ArrayList found=new ArrayList();
                Imgproc.Canny(mTmp,edges , 100, 200);
                List<MatOfPoint> contours=new ArrayList<MatOfPoint>();
                Mat hierarchy=new Mat();
                Imgproc.findContours(edges,contours,hierarchy,RETR_TREE,CHAIN_APPROX_SIMPLE);
                    for (int i=0;i<contours.size();i++){
                        double k=i;
                        int c=0;
                        double []b;

                        while (k!=-1.0){
                            b=hierarchy.get(0,(int)k);
                            k=b[2];
                            c++;
                            if (c > 5) {
                                found.add(i);
                            }
                        }
                    }
                    //Mat mTm=new Mat(rows,cols,CvType.CV_8UC1);

                Imgproc.cvtColor(edges, mRgba, Imgproc.COLOR_GRAY2RGBA, 4);
                for (int i = 0; i < found.size(); i++) {
                    Imgproc.drawContours(mRgba, contours, (int) found.get(i), new Scalar(0, 255, 0), 3);
                }*/

                break;
            case 2:
                Mat hsv2=new Mat();
                mRgba=inputFrame.rgba();
                cvtColor(mRgba,hsv2,COLOR_RGB2HSV);
                break;
            default:
                break;
        }
        return mRgba;
    }

    @Override
    public void onSensorChanged(SensorEvent event) {
        int type = event.sensor.getType();
        float data[];
        if (type == Sensor.TYPE_ACCELEROMETER) {
            data = mGData;
        } else if (type == Sensor.TYPE_MAGNETIC_FIELD) {
            data = mMData;

        } else {
            return;
        }
        for (int i = 0; i < 3; i++) {
            data[i] = event.values[i];
        }
        // 根据设备传输过来的向量数据计算倾斜矩阵mR以及旋转矩阵mI
        SensorManager.getRotationMatrix(mR, mI, mGData, mMData);
        // 根据旋转矩阵mR计算出设备的方向
        SensorManager.getOrientation(mR, mOrientation);
        /**
         * values[0]  ：azimuth 方向角，但用（磁场+加速度）得到的数据范围是（-180～180）,
         * 也就是说，0表示正北，90表示正东，180/-180表示正南，-90表示正西。
         * 而直接通过方向感应器数据范围是（0～359）360/0表示正北，90表示正东，180表示正南，270表示正西。
         values[1]  pitch 倾斜角   即由静止状态开始，前后翻转
         values[2]  roll 旋转角  即由静止状态开始，左右翻转
         */
        //手机绕z轴旋转的度数
        float azimuth = (float) Math.toDegrees(mOrientation[0]);
        //手机绕x轴旋转的度数
        float pitch = (float) Math.toDegrees(mOrientation[1]);
        //手机绕y轴旋转的度数
        float roll = (float) Math.toDegrees(mOrientation[2]);

        tv_Roll.setText("Roll"+String.valueOf(roll));
        tv_Pitch.setText("X轴"+String.valueOf(pitch));
        tv_Yaw.setText("Y轴"+String.valueOf(azimuth));

        //坐标系：android.hardware.SensorManager.getOrientation()方法所示的坐标系
        //手机旋转姿势的确定：手机屏幕向上水平放置时，手机头部向北azimuth为0度，头部向正南aimuth为180度/-180度，
        //头部向正西azimuth为-90度，头部向正东azimuth为90度，此时pitch和roll的值基本为零，因为此时手机可能不是绝对水平
        //(注意：可能是因为传感器厂家的不同，此时手机头部向正南时azimuth显示为0度，向正北时azimuth显示为180度/-180度)
        //手机屏幕水平向上放置时，手机尾部抬起至垂直于地面时，pitch的值0-90度，继续翻转至手机屏幕水平向下，pitch的值90-0度
        //手机屏幕水平向上放置时，手机头部抬起至垂直于地面时，pitch的值0值-90度，继续翻转至手机屏幕水平向下时，pitch的值-90-0度
        //手机屏幕水平向上放置时，手机右侧向上抬起至手机屏幕水平向下，roll的值：0至-180度，
        //手机左侧向上抬起至屏幕水平向下，roll的值：0-180度
    }

    @Override
    public void onAccuracyChanged(Sensor sensor, int accuracy) {

    }
}
