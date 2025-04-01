import com.acmerobotics.roadrunner.Vector2d
import org.firstinspires.ftc.teamcode.helpers.Line2d
import org.junit.Test

class SonicTests {
    @Test
    fun lineTest() {
        val line1 = Line2d(Vector2d(0.0, 0.0), Vector2d(1.0, 1.0))
        val line2 = Line2d(Vector2d(0.0, 0.0), Vector2d(1.0, -1.0))

        val intersection = line1.intersect(line2)
        println(intersection)


    }
}