package pbl3;
import robocode.*;
import robocode.Robot;
import robocode.ScannedRobotEvent;
import static robocode.util.Utils.normalRelativeAngleDegrees;
import java.awt.*;
import java.util.ArrayList;
import java.lang.Math;

//import java.awt.Color;

// API help : https://robocode.sourceforge.io/docs/robocode/robocode/Robot.html

/**
 * PBL3 - a robot by (PBL-3)
 */
public class PBL3 extends AdvancedRobot
{
	/**
	 * run: PBL3's default behavior
	 */
	private ArrayList<RobotData> enemies; // 敵の車両の情報を格納
	private ArrayList<RobotData> allies; // 味方の車両の情報を格納
	private ArrayList<RobotData> Walls; // Wallsの情報を格納
	private int targetIndex; // ターゲットのリスト番号を格納
	private boolean onTarget = false; // ターゲットが決まっている場合はtrue, 決まっていない場合はfalse
	private double bEnergy; //弾丸エネルギー
	
	double[] enemies = new double[10];

	public void run() {
		// Initialization of the robot should be put here
		init();
		// After trying out your robot, try uncommenting the import at the top,
		// and the next line:

		// setColors(Color.red,Color.blue,Color.green); // body,gun,radar

		// Robot main loop
		while(true) {
			// Replace the next 4 lines with any behavior you would like
			// 攻撃時
			if(onTarget){
				setAhead(retDistance());
				setTurnRight(retAngle());
				execute();
			}
			// 索敵時
			else if(!onTarget){

			}
		}

		// add add addd add
	}

	public void init() {
		setBodyColor(Color.blue);
		setGunColor(Color.black);
		setRadarColor(Color.yellow);
		setBulletColor(Color.green);
		setScanColor(Color.green);
	}

	/**
	 * onScannedRobot: What to do when you see another robot
	 */
	public void onScannedRobot(ScannedRobotEvent e) {
		// Replace the next line with any behavior you would like
		fire(1);
	}

	/**
	 * onHitByBullet: What to do when you're hit by a bullet
	 */
	public void onHitByBullet(HitByBulletEvent e) {
		// Replace the next line with any behavior you would like
		back(10);
	}
	
	/**
	 * onHitWall: What to do when you hit a wall
	 */
	public void onHitWall(HitWallEvent e) {
		// Replace the next line with any behavior you would like
		back(20);
	}

	/**
	 * retVelosity: 動く距離を返す
	 */
	public double retDistance() {
		double X = this.enemies.get(this.targetIndex).getX(); // ターゲットのX座標を格納
		double Y = this.enemies.get(this.targetIndex).getY(); // ターゲットのY座標を格納
		// ターゲット以外の敵車両のxy座標を格納
		double[][] xy = new double[1][1];
		int cnt=0;
		for(int i=0; i<this.enemies.size(); i++){
			if(i != targetIndex){
				xy[cnt][0] = this.enemies.get(i).getX();
				xy[cnt][1] = this.enemies.get(i).getY();
				cnt++;
			}
		}
		return 200 / (Math.pow(2*X-xy[0][0]-xy[1][0], 2) + Math.pow(2*Y-xy[0][1]-xy[1][1], 2));
	}

	/**
	 * retTheta: 右に何度回転するかの角度を返す
	 */
	public double retAngle(){
		double X = this.enemies.get(this.targetIndex).getX(); // ターゲットのX座標を格納
		double Y = this.enemies.get(this.targetIndex).getY(); // ターゲットのY座標を格納
		// ターゲット以外の敵車両のxy座標を格納
		double[][] xy = new double[1][1];
		int cnt=0;
		for(int i=0; i<this.enemies.size(); i++){
			if(i != targetIndex){
				xy[cnt][0] = this.enemies.get(i).getX();
				xy[cnt][1] = this.enemies.get(i).getY();
				cnt++;
			}
		}
		double bodyAngle = getHeading(); // 現在の車体の絶対角度を返す(0~360)
		double nume = 2*Y - xy[0][1] - xy[1][1]; // 分子
		double deno = Math.sqrt(Math.pow(2*X-xy[0][0]-xy[1][0], 2) + Math.pow(2*Y-xy[0][1]-xy[1][1], 2));
		double nextAngle = Math.toDegrees(Math.acos(nume/deno)); // 次にどこに向けるか(絶対角度: -180~180)
		nextAngle += 180; // 次にどこに向けるか(0~360)
		// 最短で目的角度に車両を回転する
		if(nextAngle-bodyAngle <= (360-nextAngle)+nextAngle){
			return nextAngle-bodyAngle;
		}else{
			return ((360-nextAngle)+nextAngle)*(-1);
		}
	}

	/**
	 * linearFire: 線形射撃
	 */
	public void linearFire() {
		double myX = getX();  //自分のx座標
		double myY = getY();  //自分のy座標
		double myD = getGunHeading();  //自分の砲台の向き(0~360)
		double aimX = this.enemies.get(this.targetIndex).x;  //狙うx座標
		double aimY = this.enemies.get(this.targetIndex).y;  //狙うy座標
		double mybV = 20 - 3 * bEnergy; //自分の弾丸の速度
		double aimD = Math.toDegrees(Math.asin(Math.sin(this.enemies.get(this.targetIndex).direction)*(this.enemies.get(this.targetIndex).velocity / mybV) + aimX - myX));  //狙う向き(0~360)

		//回転処理(引数が0~360の範囲になるように調整)
		if(aimD >= myD) {
			turnGunRight(aimD - myD);
		} else {
			turnGunLeft(myD - aimD);
		}
		
		//攻撃(円形射撃とまとめるなら削除)
		for(int i = 0; i < 10; i++){
			fire(bEnergy);
		}
	}

	// 円形予測射撃
	// cx, cy: 円の中心, 求め方はわからない.
	// mx, my: 自分の現在座標
	private void circularFire(RobotData targetData, double cx, double cy, double mx, double my, double bulletPower) {
		// ニュートン法で弾が当たるまでの時間を求める
		double tx = targetData.getX();
		double ty = targetData.getY();
		double bulletVelocity = Rules.getBulletSpeed(bulletPower);
		double omega = targetData.getVelocity() / dist(cx, cy, tx, ty);

		double t = dist(tx, ty, mx, my);
		while (f_newton(omega, tx, ty, cx, cy, mx, my, bulletVelocity, t) > 1e-3) { // 適当
			t = t - f_newton(omega, tx, ty, cx, cy, mx, my, bulletVelocity, t) / fprime_newton(omega, tx, ty, cx, cy, mx, my, bulletVelocity, t);
		}

		Complex center = new Complex(cx, cy);
		// t時刻後の相対位置
		Complex rpos = new Complex(tx, ty)
			.sub(center)
			.mul(new Complex(Math.cos(omega*t), Math.sin(omega*t)))
			.add(center)
			.sub(new Complex(mx, my));
		
		// y軸正方向から時計回りに図った角度, 単位は度
		double fireDir = (Math.PI / 2.0 - Math.acos(rpos.getRealPart() / dist(0, 0, rpos.getRealPart(), rpos.getImaginaryPart()))) * 180.0 / Math.PI;
		double currentDir = getGunHeading();
		turnGunRight(fireDir - currentDir);
		fire(bulletPower);
	}

	// circularFire内で使う関数
	// omegaは敵の等速円運動の角速度
	private double f_newton(double omega, double tx, double ty, double cx, double cy, double mx, double my, double bulletVelocity, double t) {
		double A = Math.pow(tx - cx, 2) + Math.pow(ty - cy, 2);
		double B = A;
		double C = 2.0*(tx - cx)*(cx - mx) + 2.0*(ty - cy)*(cy - my);
		double D = -2.0*(ty - cy)*(cx - mx) + 2.0*(tx - cx)*(cy - my);
		double E = -Math.pow(bulletVelocity, 2);
		double F = Math.pow(cx - mx, 2) + Math.pow(cy - my, 2);

		return A * Math.pow(Math.cos(omega*t), 2)
			+ B * Math.pow(Math.sin(omega*t), 2)
			+ C * Math.cos(omega*t)
			+ D * Math.sin(omega*t)
			+ E * Math.pow(t, 2)
			+ F;
	}

	// f_newtonの微分
	private double fprime_newton(double omega, double tx, double ty, double cx, double cy, double mx, double my, double bulletVelocity, double t) {
		double h = 1e-6; // 適当
		return (f_newton(omega, tx, ty, cx, cy, mx, my, bulletVelocity, t + h) - f_newton(omega, tx, ty, cx, cy, mx, my, bulletVelocity, t)) / h;
	}

	// (x1, y1)と(x2, y2)の距離
	private double dist(double x1, double y1, double x2, double y2) {
		return Math.sqrt(Math.pow(x1 - x2, 2) + Math.pow(y1 - y2, 2));
	}
	
	public double decideTarget(){
		double[][] enemiesxy = new double[10][10];
		int cnt=0;
		for(int i=0; i<this.enemies.size(); i++){
			enemiesxy[cnt][0] = this.enemies.get(i).getX(); //敵のx座標
			enemiesxy[cnt][1] = this.enemies.get(i).getY(); //敵のy座標
			cnt++;	
		}
		
		int cnt=0;
		for(int i=0; i<this.allies.size(); i++){
			myxy[cnt][0] = this.allies.get(i).getX(); //味方のx座標
			myxy[cnt][1] = this.allies.get(i).getY(); //味方のy座標
			cnt++;	
		}
		
		double[] distance = new double[10];

		for(int cnt=0; i<this.enemies.size(); cnt++){
			double ex=0;
			for(int i=0; i<this.allies.size(); i++){
				ex = ex + Math.sqrt((myxy[i][0]-enemiesxy[i][0])*2 + (myxy[i][1]-enemiesxy[i][1])*2);
			}
			distance[cnt] = ex;
		}
		
		double max = distance[0];
		int maxorder = 0;
		for(int i=0; i<distance.length; i++){
			if(max < distance[i]){
				max = distance[i];
				maxorder = i;
			}
		}	
		
		targetIndex = maxorder;
		onTarget = true;
	
	}
}