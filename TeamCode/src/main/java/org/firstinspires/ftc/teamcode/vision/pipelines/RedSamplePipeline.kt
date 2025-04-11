package org.firstinspires.ftc.teamcode.vision.pipelines

import org.opencv.core.Core
import org.opencv.core.CvType
import org.opencv.core.Mat
import org.opencv.core.MatOfPoint
import org.opencv.core.MatOfPoint2f
import org.opencv.core.Point
import org.opencv.core.RotatedRect
import org.opencv.core.Scalar
import org.opencv.core.Size
import org.opencv.imgproc.Imgproc
import org.openftc.easyopencv.OpenCvPipeline

class RedSamplePipeline : OpenCvPipeline() {
    var lowerYCrCb: Scalar = Scalar(0.0, 173.0, 0.0, 0.0)
    var upperYCrCb: Scalar = Scalar(255.0, 255.0, 110.0, 0.0)
    private val ycrcbMat = Mat()
    private val ycrcbBinaryMat = Mat()

    var erodeValue: Int = 0
    var dilateValue: Int = 0
    private var element: Mat? = null
    private val ycrcbBinaryMatErodedDilated = Mat()

    private val contours = ArrayList<MatOfPoint>()
    private val hierarchy = Mat()

    private val contours2f = MatOfPoint2f()
    private val contoursRotRects = ArrayList<RotatedRect?>()

    var lineColor: Scalar = Scalar(0.0, 255.0, 0.0, 0.0)
    var lineThickness: Int = 3

    private val inputRotRects = Mat()

    var lineColor1: Scalar = Scalar(0.0, 255.0, 0.0, 0.0)
    var lineThickness1: Int = 3

    private val inputMask = Mat()

    private val inputMaskContours = Mat()

    override fun processFrame(input: Mat): Mat {
        Imgproc.cvtColor(input, ycrcbMat, Imgproc.COLOR_RGB2YCrCb)
        Core.inRange(ycrcbMat, lowerYCrCb, upperYCrCb, ycrcbBinaryMat)

        ycrcbBinaryMat.copyTo(ycrcbBinaryMatErodedDilated)
        if (erodeValue > 0) {
            this.element = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT,
                Size(erodeValue.toDouble(), erodeValue.toDouble())
            )
            Imgproc.erode(ycrcbBinaryMatErodedDilated, ycrcbBinaryMatErodedDilated, element)

            element!!.release()
        }

        if (dilateValue > 0) {
            this.element = Imgproc.getStructuringElement(
                Imgproc.MORPH_RECT,
                Size(dilateValue.toDouble(), dilateValue.toDouble())
            )
            Imgproc.dilate(ycrcbBinaryMatErodedDilated, ycrcbBinaryMatErodedDilated, element)

            element!!.release()
        }

        contours.clear()
        hierarchy.release()
        Imgproc.findContours(
            ycrcbBinaryMatErodedDilated,
            contours,
            hierarchy,
            Imgproc.RETR_EXTERNAL,
            Imgproc.CHAIN_APPROX_SIMPLE
        )

        contoursRotRects.clear()
        for (points in contours) {
            contours2f.release()
            points.convertTo(contours2f, CvType.CV_32F)

            contoursRotRects.add(Imgproc.minAreaRect(contours2f))
        }

        input.copyTo(inputRotRects)
        for (rect in contoursRotRects) {
            if (rect != null) {
                val rectPoints = arrayOfNulls<Point>(4)
                rect.points(rectPoints)
                val matOfPoint = MatOfPoint(*rectPoints)

                Imgproc.polylines(
                    inputRotRects,
                    mutableListOf<MatOfPoint?>(matOfPoint),
                    true,
                    lineColor,
                    lineThickness
                )
            }
        }

        inputMask.release()
        Core.bitwise_and(input, input, inputMask, ycrcbBinaryMatErodedDilated)

        inputMask.copyTo(inputMaskContours)
        Imgproc.drawContours(inputMaskContours, contours, -1, lineColor1, lineThickness1)

        return inputRotRects
    }
}
