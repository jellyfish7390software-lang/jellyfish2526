package org.firstinspires.ftc.teamcode.offseason.math

import com.example.simulator.math.Parameterized
import com.example.simulator.math.ParameterizedObject
import com.example.simulator.math.Pose
import com.example.simulator.math.Vector

@Parameterized
data class ParameterizedCircle(val center: Vector, val radius: Double): ParameterizedObject {
    override fun solve(t: Double): Pose? {
        if (t in 0.0..1.0) {
            val angRad = t * 2*Math.PI

            val x = Math.cos(angRad) * radius + center.x
            val y = Math.sin(angRad) * radius + center.y

            return Pose(Vector(x,y))
        }
        return null
    }
}
