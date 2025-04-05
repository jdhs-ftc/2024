package org.firstinspires.ftc.teamcode.vision.pipelines;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;

public class BlueSamplePipeline extends OpenCvPipeline {

	public Scalar lowerYCrCb = new Scalar(0.0, 0.0, 148.0, 0.0);
	public Scalar upperYCrCb = new Scalar(76.0, 129.0, 255.0, 0.0);
	private Mat ycrcbMat = new Mat();
	private Mat ycrcbBinaryMat = new Mat();

	public int erodeValue = 3;
	public int dilateValue = 3;
	private Mat element = null;
	private Mat ycrcbBinaryMatErodedDilated = new Mat();

	private ArrayList<MatOfPoint> contours = new ArrayList<>();
	private Mat hierarchy = new Mat();

	private MatOfPoint2f contours2f = new MatOfPoint2f();
	private ArrayList<RotatedRect> contoursRotRects = new ArrayList<>();

	public Scalar lineColor = new Scalar(0.0, 255.0, 0.0, 0.0);
	public int lineThickness = 3;

	private Mat inputRotRects = new Mat();

	public Scalar lineColor1 = new Scalar(0.0, 255.0, 0.0, 0.0);
	public int lineThickness1 = 3;

	private Mat inputMask = new Mat();

	private Mat inputMaskContours = new Mat();

	@Override
	public Mat processFrame(Mat input) {
		Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb);
		Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat);

		ycrcbBinaryMat.copyTo(ycrcbBinaryMatErodedDilated);
		if(erodeValue > 0) {
			this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(erodeValue, erodeValue));
			Imgproc.erode(ycrcbBinaryMatErodedDilated, ycrcbBinaryMatErodedDilated, element);

			element.release();
		}

		if(dilateValue > 0) {
			this.element = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, new Size(dilateValue, dilateValue));
			Imgproc.dilate(ycrcbBinaryMatErodedDilated, ycrcbBinaryMatErodedDilated, element);

			element.release();
		}

		contours.clear();
		hierarchy.release();
		Imgproc.findContours(ycrcbBinaryMatErodedDilated, contours, hierarchy, Imgproc.RETR_EXTERNAL, Imgproc.CHAIN_APPROX_SIMPLE);

		contoursRotRects.clear();
		for(MatOfPoint points : contours) {
			contours2f.release();
			points.convertTo(contours2f, CvType.CV_32F);

			contoursRotRects.add(Imgproc.minAreaRect(contours2f));
		}

		input.copyTo(inputRotRects);
		for(RotatedRect rect : contoursRotRects) {
			if(rect != null) {
				Point[] rectPoints = new Point[4];
				rect.points(rectPoints);
				MatOfPoint matOfPoint = new MatOfPoint(rectPoints);

				Imgproc.polylines(inputRotRects, Collections.singletonList(matOfPoint), true, lineColor, lineThickness);
			}
		}

		inputMask.release();
		Core.bitwise_and(input, input, inputMask, ycrcbBinaryMatErodedDilated);

		inputMask.copyTo(inputMaskContours);
		Imgproc.drawContours(inputMaskContours, contours, -1, lineColor1, lineThickness1);

		return inputRotRects;
	}
}
