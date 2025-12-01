package org.firstinspires.ftc.teamcode.purepursuit.math

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.canvas.Canvas
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Vector2d
import com.arcrobotics.ftclib.controller.PIDController
import com.qualcomm.robotcore.hardware.PIDCoefficients
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.Robot
import org.firstinspires.ftc.teamcode.comp1.roadrunner.MecanumDrive
import org.firstinspires.ftc.teamcode.offseason.math.ParameterizedCircle
import org.firstinspires.ftc.teamcode.purepursuit.MecanumDrivePurePursuit
import org.firstinspires.ftc.teamcode.purepursuit.pid.SquIDController
import java.lang.annotation.Inherited
import java.util.Collections
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin

class PurePursuit(drive: MecanumDrivePurePursuit) {
    fun Pose2d.toPose(): Pose {
        return Pose(this.position.x, this.position.y, this.heading.toDouble())
    }

    companion object {
        @JvmField var builder: BezierBuilder = BezierBuilder()
        @JvmField var pathBuilder: BezierPathBuilder = BezierPathBuilder()

        @JvmField var drawPathsInBuild: Boolean = true
    }

    @JvmField var mecDrive = drive
    @JvmField var localizer = mecDrive.localizer!!

    @JvmField var pose = localizer.pose.toPose()

    @JvmField var targetPose = Pose()

    @JvmField var searchRad: Double = 10.0
    @JvmField var searchRadius = ParameterizedCircle(pose.vec(), searchRad)

    @JvmField var lastT = 0.0


    private var epsilon = 6e-5
    private var stepSize = 1e-5
    private var checkEpsilon = 1e-4
    private var sleepTime: Long = 50

    @JvmField var hasReachedDestination = false

    fun updateSearchRadius(radius: Double) {
        searchRad = radius
        pose = localizer.pose.toPose()
        localizer = mecDrive.localizer!!
        searchRadius = ParameterizedCircle(pose.vec(), radius)
    }

    object Defaults {
        val posTolerance = 1.0
        val hTolerance = 5.0.toRadians()
    }

    @JvmField var kSQx = 0.5
    @JvmField var kSQy = 0.75

    var xSquid: SquIDController = SquIDController(kSQx)
    var ySquid: SquIDController = SquIDController(kSQy)
    @JvmField var hPID: PIDCoefficients = PIDCoefficients(1.0,0.08,0.05)

    var hController = PIDController(hPID.p, hPID.i, hPID.d)

    @JvmField var targetList = Collections.emptyList<ParametricPose>()

    @JvmField var rotX = 0.0
    @JvmField var rotY = 0.0
    @JvmField var powH = 0.0


    fun updatePose() {
        localizer = mecDrive.localizer!!
        pose = localizer.pose.toPose()
    }
    fun clip (value: Double, lowerBound: Double, upperBound: Double): Double {
        if (value > upperBound) return upperBound
        else if (value < lowerBound) return lowerBound
        return value
    }

    fun normalizeAngleError(target: Double, current: Double): Double {
        val error = (target - current + Math.PI) % (2 * Math.PI) - Math.PI
        return if (error < -Math.PI) error + (2 * Math.PI) else error
    }

    fun singlePIDtoPoint(pose: Pose) {
        xSquid.setSquID(kSQx)
        ySquid.setSquID(kSQy)
        hController.setPID(hPID.p, hPID.i, hPID.d)

        this.targetPose = pose

        updateSearchRadius(searchRad)

        val heading: Double = this.pose.h

        val x = xSquid.compute(pose.x - this.pose.x)
        val y = ySquid.compute(pose.y - this.pose.y)

        var rotX = x * cos(-heading) - y * sin(-heading)
        val rotY = x * sin(-heading) + y * cos(-heading)

        rotX *= 1.1

        val hError = AngleUnit.normalizeRadians(pose.h - heading)

        val h: Double = hController.calculate(0.0, hError)

        this.rotX = rotX
        this.rotY = rotY
        this.powH = h

        mecDrive.setDrivePowers(PoseVelocity2d(Vector2d(rotX, rotY), h))
        updatePose()
    }

    fun within(currentPos: Double, target: Double, tolerance: Double) = (abs(target - currentPos) <= tolerance)
    fun withinHeading(current: Double, target: Double, tolerance: Double): Boolean {
        val error = AngleUnit.normalizeRadians(target - current)
        return abs(error) <= tolerance
    }
    fun notWithinTolerance(pose: Pose, target: Pose, xyTol: Double, hTol: Double): Boolean {
        return !within(pose.x, target.x, xyTol) || !within(pose.y, target.y, xyTol) || !withinHeading(pose.h, target.h, hTol)
    }


    //TODO: Pure-Pursuit Functions and Algorithms

    fun calcIntersections(curve: Bezier, searchRad: ParameterizedCircle): MutableList<ParametricPose> {

        fun tEquation(t: Double): Double {
            val pos = curve.solveParametric(t) ?: throw IllegalArgumentException(Bezier.WARNING + " Also make sure t is in range.")
            return (pos.x - searchRad.center.x).pow(2) + (pos.y - searchRad.center.y).pow(2) - searchRad.radius.pow(2)
        }

        fun tEquationDerivative(t: Double): Double {
            val pos = curve.solveParametric(t) ?: throw IllegalArgumentException(Bezier.WARNING + " Also make sure t is in range.")
            val derivative = curve.bezierCurveDerivative(t) ?: return 0.0
            return 2 * ((pos.x - searchRad.center.x) * derivative.x + (pos.y - searchRad.center.y) * derivative.y)
        }

        fun findSignChanges(): List<Double> {
            val step = 0.01
            val signChangePoints = mutableListOf<Double>()

            var prevT = 0.0
            var prevValue = tEquation(prevT)

            var t = step
            while (t <= 1.0) {
                val value = tEquation(t)

                if (prevValue * value <= 0) {
                    signChangePoints.add((prevT + t) / 2)
                }

                prevT = t
                prevValue = value
                t += step
            }
            return signChangePoints
        }

        fun newtonRaphson(guess: Double, maxIter: Int = 30): Double {
            var t = guess
            repeat(maxIter) {
                val derivative = tEquationDerivative(t)
                if (abs(derivative) < 1E-6) return t
                t -= tEquation(t) / derivative
                t = Maths.clamp(t, 0.0, 1.0)
            }
            return t
        }

        fun iteration(): List<ParametricPose> {
            val tS = findSignChanges().map { newtonRaphson(it) }
            return tS.mapNotNull { curve.solveParametric(it) }
        }

        val test = iteration().toMutableList()
        val result = mergeClosePoints(test, 5 * epsilon).toMutableList()
        return result
    }


    fun mergeClosePoints(points: List<ParametricPose>, threshold: Double): List<ParametricPose> {
        val merged = mutableListOf<ParametricPose>()

        for (point in points) {
            val close = merged.find { Maths.dist(it.vec(), point.vec()) < threshold }
            if (close == null) {
                merged.add(point)
            }
        }
        return merged
    }

    fun calculateTargetPose(path: Bezier): ParametricPose {
        // Reset lastT if starting a new path
        if (lastT <= 0.0 || lastT >= 1.0) {
            lastT = 0.0
        }

        targetList = calcIntersections(path, searchRadius)

        val robotPose = this.pose
        val distToEnd = Maths.dist(robotPose, path.end().toPose())

        // Candidate: nearest intersection ahead of lastT
        val candidate = targetList
            .filter { it.t >= lastT - 1e-6 }
            .minByOrNull { Maths.dist(robotPose, it.toPose()) }

        val targetPose = when {
            targetList.isEmpty() -> path.end()
            distToEnd <= 2.0 -> path.end()
            candidate != null -> candidate
            else -> path.end()
        }

        // Adjust heading
        val adjustedPose = ParametricPose(
            targetPose.x,
            targetPose.y,
            targetPose.h + Math.PI,
            targetPose.t
        )

        // Update lastT only if we advanced along intersections
        if (candidate != null) {
            lastT = maxOf(lastT, adjustedPose.t)
        }

        return adjustedPose
    }


    fun calculateTargetPose(path: Bezier, pose: Pose): ParametricPose {
        val targetList = calcIntersections(path, searchRadius)

        val distToEnd = Maths.dist(pose, path.end().toPose())
        val distToStart = Maths.dist(pose, path.start().toPose())

        return when {
            targetList.isEmpty() -> if (distToEnd >= distToStart) path.end() else path.start()
            distToEnd <= 2.0 -> path.end()
            else -> targetList.maxByOrNull { path.findT(it) } ?: path.end()
        }
    }


    fun calculateTargetPose(path: BezierPath): ParametricPose {
        val targetList = path.beziers.flatMap { calcIntersections(it, searchRadius) }

        val distToEnd = Maths.dist(pose, path.end().toPose())

        return when {
            targetList.isEmpty() -> path.end()
            distToEnd <= 2.0 -> path.end()
            else -> targetList.maxByOrNull { path.findT(it) } ?: path.end()
        }
    }

    fun followPathSingle(path: Bezier, packet: TelemetryPacket) {
        var targetPose = calculateTargetPose(path)

        packet.put("TargetX", targetPose.x)
        packet.put("TargetY", targetPose.y)
        packet.put("TargetH", targetPose.h)
        packet.put("TargetT", targetPose.t)

        targetPose = calculateTargetPose(path)

        this.targetPose = targetPose.toPose()

        updateSearchRadius(searchRad)

        singlePIDtoPoint(targetPose.toPose())

        packet.put("CurrentX", pose.x)
        packet.put("CurrentY", pose.y)
        packet.put("CurrentH", pose.h)

        packet.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
        packet.put("SearchRadius", searchRad)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }
    fun followPathSingle(path: BezierPath, packet: TelemetryPacket) {
        if (!path.isFollowable()) return
        var targetPose = calculateTargetPose(path)

        val targetHeading = if (path.headingTargetsExist()) {
            path.headingTargets.filter { it.t > targetPose.t }
                .minByOrNull { it.t }
                ?.heading ?: targetPose.h
        } else {
            targetPose.h
        }

        targetPose = ParametricPose(targetPose, targetHeading)

        singlePIDtoPoint(targetPose.toPose())

        updateSearchRadius(searchRad)

        packet.put("TargetX", targetPose.x)
        packet.put("TargetY", targetPose.y)
        packet.put("TargetH", targetPose.h)
        packet.put("TargetT", targetPose.t)

        packet.put("CurrentX", pose.x)
        packet.put("CurrentY", pose.y)
        packet.put("CurrentH", pose.h)

        packet.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
        packet.put("SearchRadius", searchRad)

        FtcDashboard.getInstance().sendTelemetryPacket(packet)
    }

    fun followPath(path: Bezier, xyTol: Double, hTol: Double, packet: TelemetryPacket) {
        // Reset state for a new path
        lastT = 0.0
        hasReachedDestination = false

        var targetPose = calculateTargetPose(path)

        packet.put("TargetX", targetPose.x)
        packet.put("TargetY", targetPose.y)
        packet.put("TargetH", targetPose.h)
        packet.put("TargetT", targetPose.t)

        while (!hasReachedDestination && notWithinTolerance(pose, targetPose.toPose(), xyTol, hTol)) {
            targetPose = calculateTargetPose(path)

            this.targetPose = targetPose.toPose()

            updateSearchRadius(searchRad)

            singlePIDtoPoint(targetPose.toPose())

            packet.put("CurrentX", pose.x)
            packet.put("CurrentY", pose.y)
            packet.put("CurrentH", pose.h)

            packet.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
            packet.put("SearchRadius", searchRad)
            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }


        hasReachedDestination = true
    }

    fun followPath(path: Bezier, packet: TelemetryPacket) = followPath(path, Defaults.posTolerance, Defaults.hTolerance, packet)

    fun followPath(path: BezierPath, xyTol: Double, hTol: Double, packet: TelemetryPacket) {
        if (!path.isFollowable()) return

        var targetPose = calculateTargetPose(path)
        if (!drawPathsInBuild) path.beziers.forEach { it.draw(packet.fieldOverlay()) }

        while (!hasReachedDestination && notWithinTolerance(pose, targetPose.toPose(), xyTol, hTol)) {
            targetPose = calculateTargetPose(path)

            val targetHeading = if (path.headingTargetsExist()) {
                path.headingTargets.filter { it.t > targetPose.t }
                    .minByOrNull { it.t }
                    ?.heading ?: targetPose.h
            } else {
                targetPose.h
            }

            targetPose = ParametricPose(targetPose, targetHeading)

            singlePIDtoPoint(targetPose.toPose())

            updateSearchRadius(searchRad)

            packet.put("TargetX", targetPose.x)
            packet.put("TargetY", targetPose.y)
            packet.put("TargetH", targetPose.h)
            packet.put("TargetT", targetPose.t)

            packet.put("CurrentX", pose.x)
            packet.put("CurrentY", pose.y)
            packet.put("CurrentH", pose.h)

            packet.put("DistanceToTarget", Maths.dist(pose, targetPose.toPose()))
            packet.put("SearchRadius", searchRad)

            FtcDashboard.getInstance().sendTelemetryPacket(packet)
        }
        hasReachedDestination = true
    }

    fun followPath(path: BezierPath, packet: TelemetryPacket) = followPath(path, Defaults.posTolerance, Defaults.hTolerance, packet)

    fun cancel() {
        hasReachedDestination = true
        mecDrive.setPowers()
    }
    fun drawRobot(c: Canvas, t: Pose2d) {
        val ROBOT_RADIUS = 9.0

        c.setStrokeWidth(1)
        c.strokeCircle(t.position.x, t.position.y, ROBOT_RADIUS)

        val halfv = t.heading.vec().times(0.5 * ROBOT_RADIUS)
        val p1 = t.position.plus(halfv)
        val p2 = p1.plus(halfv)
        c.strokeLine(p1.x, p1.y, p2.x, p2.y)
    }
}

private fun Double.toRadians(): Double {
    return Math.toRadians(this)
}

fun main() {
    val bezier = PurePursuit.builder
        .addControlPoint(60.4, 23.0)
        .addControlPoint(-61.0, -25.0)
        .addControlPoint(-50.3, -75.7)
        .addControlPoint(94.7, -62.0)
        .addControlPoint(-16.0, 43.7)
        .build()

//
//    println(PurePursuit.calcIntersections(bezier, PurePursuit.searchRadius))
//    println(PurePursuit.calculateTargetPose(bezier))

}

