import processing.core.*;
import processing.serial.*;
import processing.opengl.*;
import toxi.geom.*;
import toxi.geom.Vec3D.Axis;
import toxi.math.InterpolateStrategy;
import toxi.processing.*;

import java.nio.ByteBuffer;
import java.io.*;

import org.mavlink.IMAVLinkMessage;
import org.mavlink.MAVLinkReader;
import org.mavlink.messages.IMAVLinkMessageID;
import org.mavlink.messages.MAVLinkMessage;
import org.mavlink.messages.ardupilotmega.msg_attitude_quaternion;

public class Main extends PApplet {

	// NOTE: requires ToxicLibs to be installed in order to run properly.
	// 1. Download from http://toxiclibs.org/downloads
	// 2. Extract into [userdir]/Processing/libraries
	// (location may be different on Mac/Linux)
	// 3. Run and bask in awesomeness

	/**
	 * 
	 */
	private static final long serialVersionUID = -6560860671868622778L;

	ToxiclibsSupport gfx;

	Serial port; // The serial port
	char[] teapotPacket = new char[18]; // InvenSense Teapot packet
	int serialCount = 0; // current packet byte position
	int aligned = 0;
	int interval = 0;

	float[] q = new float[4];
	byte[] b = new byte[4];
	Quaternion quat = new Quaternion(1, 0, 0, 0);

	float[] gravity = new float[3];
	float[] euler = new float[3];
	float[] ypr = new float[3];

	ByteBuffer data;
	PipedInputStream pis = new PipedInputStream();
	PipedOutputStream pos;
	Quaternion zeroQuat;
	
	public void setup() {
		// 300px square viewport using OpenGL rendering
		size(300, 300, OPENGL);
		gfx = new ToxiclibsSupport(this);

		// setup lights and antialiasing
		lights();
		smooth();

		// display serial port list for debugging/clarity
		println(Serial.list());

		// get the first available port (use EITHER this OR the specific port
		// code below)
		// String portName = Serial.list()[1];

		// get a specific serial port (use EITHER this OR the first-available
		// code above)
		String portName = "COM3";

		// open the serial port
		port = new Serial(this, portName, 230400);

		// send single character to trigger DMP init/start
		// (expected by MPU6050_DMP6 example Arduino sketch)
		port.write('r');

		try {
			pos = new PipedOutputStream(pis);
		} catch (IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		zeroQuat = new Quaternion(1, 0, 0, 0);
	}

	public void draw() {
		if (millis() - interval > 1000) {
			// resend single character to trigger DMP init/start
			// in case the MPU is halted/reset while applet is running
			// port.write('r');
			interval = millis();
		}

		// black background
		background(0);

		// translate everything to the middle of the viewport
		pushMatrix();
		translate(width / 2, height / 2);

		// 3-step rotation from yaw/pitch/roll angles (gimbal lock!)
		// ...and other weirdness I haven't figured out yet
		// rotateY(-ypr[0]);
		// rotateZ(-ypr[1]);
		// rotateX(-ypr[2]);

		// toxiclibs direct angle/axis rotation from quaternion (NO gimbal
		// lock!)
		// (axis order [1, 3, 2] and inversion [-1, +1, +1] is a consequence of
		// different coordinate system orientation assumptions between
		// Processing
		// and InvenSense DMP)
		float[] axis = quat.toAxisAngle();
		rotate(axis[0], -axis[1], axis[3], axis[2]);

		// draw main body in red
		fill(255, 0, 0, 200);
		box(10, 10, 200);

		// draw front-facing tip in blue
		fill(0, 0, 255, 200);
		pushMatrix();
		translate(0, 0, -120);
		rotateX(PI / 2);
		drawCylinder(0, 20, 20, 8);
		popMatrix();

		// draw wings and tail fin in green
		fill(0, 255, 0, 200);
		beginShape(TRIANGLES);
		vertex(-100, 2, 30);
		vertex(0, 2, -80);
		vertex(100, 2, 30); // wing top layer
		vertex(-100, -2, 30);
		vertex(0, -2, -80);
		vertex(100, -2, 30); // wing bottom layer
		vertex(-2, 0, 98);
		vertex(-2, -30, 98);
		vertex(-2, 0, 70); // tail left layer
		vertex(2, 0, 98);
		vertex(2, -30, 98);
		vertex(2, 0, 70); // tail right layer
		endShape();
		beginShape(QUADS);
		vertex(-100, 2, 30);
		vertex(-100, -2, 30);
		vertex(0, -2, -80);
		vertex(0, 2, -80);
		vertex(100, 2, 30);
		vertex(100, -2, 30);
		vertex(0, -2, -80);
		vertex(0, 2, -80);
		vertex(-100, 2, 30);
		vertex(-100, -2, 30);
		vertex(100, -2, 30);
		vertex(100, 2, 30);
		vertex(-2, 0, 98);
		vertex(2, 0, 98);
		vertex(2, -30, 98);
		vertex(-2, -30, 98);
		vertex(-2, 0, 98);
		vertex(2, 0, 98);
		vertex(2, 0, 70);
		vertex(-2, 0, 70);
		vertex(-2, -30, 98);
		vertex(2, -30, 98);
		vertex(2, 0, 70);
		vertex(-2, 0, 70);
		endShape();

		popMatrix();
	}
	
	MAVLinkReader reader = new MAVLinkReader(new DataInputStream(pis), IMAVLinkMessage.MAVPROT_PACKET_START_V10);
	MAVLinkMessage msg;
	float oldValue = 0;
	public void serialEvent(Serial port) throws IOException {
		interval = millis();
		// println(port.available());
		while (port.available() > 0) {
			int ch = port.read();
			// println(ch);

			pos.write(ch);
			msg = reader.getNextMessageWithoutBlocking();
			if (msg != null) {
				msg_attitude_quaternion m = (msg_attitude_quaternion) msg;
				if (msg.messageType == IMAVLinkMessageID.MAVLINK_MSG_ID_ATTITUDE_QUATERNION) {

					if (oldValue == 0.0f && m.rollspeed == 1.0f) {
						zeroQuat.set(m.q1, m.q2, m.q3, m.q4);
						println(zeroQuat);										
					}
					oldValue = m.rollspeed;
					
					quat.set(m.q1, m.q2, m.q3, m.q4);
					quat = quat.multiply(zeroQuat.getConjugate());
					//quat = zeroQuat.getConjugate().multiply(quat);
					//quat = quat.getConjugate().multiply(zeroQuat);
					//quat = quat.multiply(zeroQuat.getConjugate());
				}

			}
		}

		// set our toxilibs quaternion to new data
		// quat.set(q[0], q[1], q[2], q[3]);

	}

	void drawCylinder(float topRadius, float bottomRadius, float tall, int sides) {
		float angle = 0;
		float angleIncrement = TWO_PI / sides;
		beginShape(QUAD_STRIP);
		for (int i = 0; i < sides + 1; ++i) {
			vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
			vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
			angle += angleIncrement;
		}
		endShape();

		// If it is not a cone, draw the circular top cap
		if (topRadius != 0) {
			angle = 0;
			beginShape(TRIANGLE_FAN);

			// Center point
			vertex(0, 0, 0);
			for (int i = 0; i < sides + 1; i++) {
				vertex(topRadius * cos(angle), 0, topRadius * sin(angle));
				angle += angleIncrement;
			}
			endShape();
		}

		// If it is not a cone, draw the circular bottom cap
		if (bottomRadius != 0) {
			angle = 0;
			beginShape(TRIANGLE_FAN);

			// Center point
			vertex(0, tall, 0);
			for (int i = 0; i < sides + 1; i++) {
				vertex(bottomRadius * cos(angle), tall, bottomRadius * sin(angle));
				angle += angleIncrement;
			}
			endShape();
		}
	}
}
