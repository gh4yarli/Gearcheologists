package org.firstinspires.ftc.teamcode;

public class ShootingTriangle {
    public class Triangle {
        public int shootingVel;
        public Triangle(int shootingVel) {
            this.shootingVel = shootingVel;
        }
    }
    public Triangle bigTriangle = new Triangle(1380);
    public Triangle smallTriangle = new Triangle(1780);
}
