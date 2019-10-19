package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;
import android.util.Log;

import com.vuforia.Image;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.firstinspires.ftc.teamcode.ConceptVuforiaSkyStoneNavigation;

import java.util.ArrayList;
import java.util.List;

/**
* GripPipeline class.
*
* <p>An OpenCV pipeline generated by GRIP.
*
* @author GRIP
*/
public class GripPipeline {

	//Outputs
	private Mat blurOutput = new Mat();
	private Mat rgbThresholdOutput = new Mat();
	private ArrayList<MatOfPoint> findContoursOutput = new ArrayList<MatOfPoint>();
	private final String TAG = "GripPipeLine";




	/**
	 * This is the primary method that runs the entire pipeline and updates the outputs.
	 * @return
	 */
	public Point process(Mat source0) {
		// Step Blur0:
		Mat blurInput = source0;
		BlurType blurType = BlurType.get("Box Blur");
		double blurRadius = 54.95495495495496;
		blur(blurInput, blurType, blurRadius, blurOutput);

		// Step RGB_Threshold0:
		Mat rgbThresholdInput = blurOutput;
		double[] rgbThresholdRed = {169.69424460431657, 255.0};
		double[] rgbThresholdGreen = {0.0, 255.0};
		double[] rgbThresholdBlue = {0.0, 91.86868686868688};
		rgbThreshold(rgbThresholdInput, rgbThresholdRed, rgbThresholdGreen, rgbThresholdBlue, rgbThresholdOutput);

		// Step Find_Contours0:
		Mat findContoursInput = rgbThresholdOutput;
		boolean findContoursExternalOnly = false;
		findContours(findContoursInput, findContoursExternalOnly, findContoursOutput);


		return FindClosePoint(source0.size());
	}

	/**
	 * This method is a generated getter for the output of a Blur.
	 * @return Mat output from Blur.
	 */
	public Mat blurOutput() {
		return blurOutput;
	}

	/**
	 * This method is a generated getter for the output of a RGB_Threshold.
	 * @return Mat output from RGB_Threshold.
	 */
	public Mat rgbThresholdOutput() {
		return rgbThresholdOutput;
	}

	/**
	 * This method is a generated getter for the output of a Find_Contours.
	 * @return ArrayList<MatOfPoint> output from Find_Contours.
	 */
	public ArrayList<MatOfPoint> findContoursOutput() {
		return findContoursOutput;
	}
	public ArrayList<org.opencv.core.Point> findContoursArray(){
		ArrayList<org.opencv.core.Point> r = new ArrayList<>();
		for (MatOfPoint p : findContoursOutput){
			r.addAll(p.toList());
		}

		return r;
	}



	/**
	 * An indication of which type of filter to use for a blur.
	 * Choices are BOX, GAUSSIAN, MEDIAN, and BILATERAL
	 */
	enum BlurType{
		BOX("Box Blur"), GAUSSIAN("Gaussian Blur"), MEDIAN("Median Filter"),
			BILATERAL("Bilateral Filter");

		private final String label;

		BlurType(String label) {
			this.label = label;
		}

		public static BlurType get(String type) {
			if (BILATERAL.label.equals(type)) {
				return BILATERAL;
			}
			else if (GAUSSIAN.label.equals(type)) {
			return GAUSSIAN;
			}
			else if (MEDIAN.label.equals(type)) {
				return MEDIAN;
			}
			else {
				return BOX;
			}
		}

		@Override
		public String toString() {
			return this.label;
		}
	}

	/**
	 * Softens an image using one of several filters.
	 * @param input The image on which to perform the blur.
	 * @param type The blurType to perform.
	 * @param doubleRadius The radius for the blur.
	 * @param output The image in which to store the output.
	 */
	private void blur(Mat input, BlurType type, double doubleRadius,
		Mat output) {
		int radius = (int)(doubleRadius + 0.5);
		int kernelSize;
		switch(type){
			case BOX:
				kernelSize = 2 * radius + 1;
				Imgproc.blur(input, output, new Size(kernelSize, kernelSize));
				break;
			case GAUSSIAN:
				kernelSize = 6 * radius + 1;
				Imgproc.GaussianBlur(input,output, new Size(kernelSize, kernelSize), radius);
				break;
			case MEDIAN:
				kernelSize = 2 * radius + 1;
				Imgproc.medianBlur(input, output, kernelSize);
				break;
			case BILATERAL:
				Imgproc.bilateralFilter(input, output, -1, radius, radius);
				break;
		}
	}

	
	private void rgbThreshold(Mat input, double[] red, double[] green, double[] blue,
		Mat out) {
		Imgproc.cvtColor(input, out, Imgproc.COLOR_BGR2RGB);
		Core.inRange(out, new Scalar(red[0], green[0], blue[0]),
			new Scalar(red[1], green[1], blue[1]), out);
	}

	
	private void findContours(Mat input, boolean externalOnly,
		List<MatOfPoint> contours) {
		Mat hierarchy = new Mat();
		contours.clear();
		int mode;
		if (externalOnly) {
			mode = Imgproc.RETR_EXTERNAL;
		}
		else {
			mode = Imgproc.RETR_LIST;
		}
		int method = Imgproc.CHAIN_APPROX_SIMPLE;
		Imgproc.findContours(input, contours, hierarchy, mode, method);
	}
	// Edited by Itay Shalev
	
	public Point FindClosePoint(Size size){
		double w = size.width/2;
		double h = size.height;
		double s = 1000000;
		double currDist = 0; // The current distance from the robot's position to the current point.
		int si   = 0;
		Point p = null;
		ArrayList<Point> arr = findContoursArray();
		Log.d(TAG,"arraySize = " +  arr.size());
		if (arr != null && arr.size()!= 0 ) {
			for (int i = 0; i < arr.size(); i++) {
				// Get distance from current position to current point
				currDist = Math.sqrt(Math.pow(w - arr.get(i).x, 2) + Math.pow(h - arr.get(i).y, 2)); // Get distance from current position to current point
				if (currDist < s) {
					s = currDist;
					si = i;
				}
			}
			return arr.get(si);
		 } else {
			return p;
		}
	}


	/*
	* The function gets a Vuforia Image class object, and converts it to an openCV Mat object.
	*/
	public Mat ImageToMat(Image input)
	{
		Bitmap bm = Bitmap.createBitmap(input.getWidth(), input.getHeight(), Bitmap.Config.RGB_565);
		bm.copyPixelsFromBuffer(input.getPixels());

		//put the image into a MAT for OpenCV
		Mat tmp = new Mat(input.getWidth(), input.getHeight(), CvType.CV_8UC4);
		Utils.bitmapToMat(bm, tmp);

		return tmp;

	}
}

