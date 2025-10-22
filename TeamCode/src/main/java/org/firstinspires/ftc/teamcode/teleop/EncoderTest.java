@TeleOp(name="Encoder Test", group="Test")
public class EncoderTest extends OpMode {
    DcMotor fl, fr, bl, br;

    @Override
    public void init() {
        fl = hardwareMap.dcMotor.get("FrontLeft");
        fr = hardwareMap.dcMotor.get("FrontRight");
        bl = hardwareMap.dcMotor.get("BackLeft");
        br = hardwareMap.dcMotor.get("BackRight");

        for (DcMotor m : new DcMotor[]{fl, fr, bl, br}) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            m.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    @Override
    public void loop() {
        telemetry.addData("FL", fl.getCurrentPosition());
        telemetry.addData("FR", fr.getCurrentPosition());
        telemetry.addData("BL", bl.getCurrentPosition());
        telemetry.addData("BR", br.getCurrentPosition());
        telemetry.update();
    }
}
