using System;
using System.Numerics;
using G = Godot;
using GD = Godot.GD;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Factorization;
using MathNet.Numerics.LinearAlgebra.Double;
using Matrix = MathNet.Numerics.LinearAlgebra.Matrix<double>;
using static Draw;

/*
-----------------------------------------------------------------

# Units
 - [M] : g
 - [L] : mm
 - [T] : s

-----------------------------------------------------------------
*/

// templates
// GD.Print($"ori : {ori.toString()}");

public partial class Main : G.Control {

        // control parameters
    public static float gain = 1.0F;
    public static float Kp = -0.4F*gain; 
    public static float Kd = -0.4F*gain;
    public static float Ki = -0.0F*gain;
    public static Quat target = new Quat(1,0,0,0);

    public struct Input{
        // 0:cg 1:design
        public static bool MBL = false;//mouse light button
        public static bool MBR = false;// \\   right
        public static bool MBM = false;// \\   middle
        public static float[] MWV = new float[2]{0,0}; //wheel
        public static Godot.Vector2[] MMV = new G.Vector2[2]; //mouse
        public static Godot.Vector2 camDiff = new Godot.Vector2(242,145);
        public static float[] scale = new float[2]{1,0.000001F};
        public static bool isMouseInside = false;
        public static bool isDragging = false;
    }
    float FOV = (float)toRad(50); //rad
    public static float[] camAngl = new float[2]{0,0};
    float sensit = 3F;
    public static float camd = 800F;

        // physics constants
    //static float cd = 60; //mm
    //static float ch = 85;

        // center of gravity & moment of inertia of cansat [g*mm*mm] copy and paste from FreeCAD
    static readonly float cm = 254.87000000000003F;
    static readonly float COMx = 0.40914837195780923F;
    static readonly float COMy = -0.41547512786331864F;
    static readonly float COMz = -46.58278965240623F;
    static readonly double Ixx = 747626.041145661;
    static readonly double Iyy = 733198.4770532331;
    static readonly double Izz = 133509.29448540072;
    static readonly double Ixy = 9988.67491157437;
    static readonly double Ixz = 9796.290439624201;
    static readonly double Iyz = -8747.89672499152;

    static readonly Vec3 r = new Vec3(COMx,COMy,COMz); // vector from forcePoint(centerOfRotation=origin) to centerOfGravity [local]
    static readonly Matrix moi = Matrix.Build.DenseOfArray(new double[,]{
            {Ixx,Ixy,Ixz},
            {Ixy,Iyy,Iyz},
            {Ixz,Iyz,Izz}
            });
    // eigen value decomposition 
    static readonly Evd<double> ev = moi.Evd();
    // primary MOI
    static readonly Vec3 cmoi_p = new Vec3((float)ev.EigenValues[0].Real,
            (float)ev.EigenValues[1].Real,
            (float)ev.EigenValues[2].Real
            );

    static readonly Vec3 cmoi = new Vec3(
            (float)Ixx,
            (float)Iyy,
            (float)Izz
            );



    //Vec3 cmoi = new Vec3(cm*cd*cd/16 + cm*ch*ch/12 , cm*cd*cd/16 + cm*ch*ch/12 , cm*cd*cd/8); cylinder's PMOI
        // reaction wheels
    //static float wd1 = 15; //mm smaller diameter ( hole )
    //static float wd2 = 40; //mm
    //static float wh = 2; //mm
    //static float wm = 24; //g
    //static float wmoi1 = wm*(wd2*wd2 + wd1*wd1)/16 + wm*wh*wh/12; // around the axis perpendicular to rotation axis
    //static float wmoi2 = wm*(wd2*wd2 + wd1*wd1)/8; // around rotation axis

    static readonly float normwav = 100;
    static readonly float restoringFactor = 1.0F/10.0F;
    static readonly float wmoi1 = 2865.2742591355664F; // g*mm^2  around the axis perpendicular to rotation axis
    static readonly float wmoi2 = 4713.268273623705F;// around rotation axis
    // max 
    static readonly float maxwav = 200; // rad/s
    // torque [M*L^2*T^-2] [g*mm^2*s^-2]
    static readonly float maxtorque = 12448130; //11.1V
    // motor :  MT2204 2300KV : Kv 2300 * 2π / 60 (rad/s)/V ~ 241 rad/s/V
    //          Kt : (N*m)/A  Kt = Kv^-1
    //          max torque = Kt * Imax = 1/241 * 3 [N*m] = 1000^3 * 1.233 / 241 [torque]
    //          # torque = W/(rad/s)
    static Vec3 wav = new Vec3(); // wheel angular velocity

    // 0:current 1:previous
    public static G.Vector2[] rpos = new G.Vector2[2]; // position on rect
    public static Vec3 pos = new Vec3(); // global
    public static Quat[] ori = new Quat[2]; // global
    public static Vec3[] oriv = new Vec3[2];

    public static Vec3 vel = new Vec3(); // global
    public static Vec3 avel = new Vec3(); // global

    public static Vec3 error = new Vec3(); // error of orientation
    public static Vec3 integral = new Vec3(); // integral of error


    double time = 0;
    bool f2 = true;
    bool doControl = false;

    Vec3 force = new Vec3(0.00F,0.0F,0.0F); // global
    Vec3 impulse = new Vec3(0.00F,0.0F,0.00F); // global
    Vec3 torque = new Vec3(0,0,0); // local !!
    //Vec3 torqueG = new Vec3(0,0,0); // local !!
    // x z y

    Vec3 deltaV =  new Vec3();
    G.Vector2 v2 =  new G.Vector2();
    G.Quaternion quat = new G.Quaternion();
    G.Transform3D trans = G.Transform3D.Identity;
    //node
    G.Node3D cansat;
    public static G.SubViewportContainer svc;
    G.Camera3D cam;
    G.ColorRect cr;
    G.RigidBody3D rb;
    G.AnimatableBody3D ab;
    G.MeshInstance3D vo;

    public static float scale = 1;
    // Called when the node enters the scene tree for the first time.
    public override void _Ready(){
        // get nodes
        rb = (G.RigidBody3D)GetNode(".//svc/sv/Node3D/rb");
        ab = (G.AnimatableBody3D)GetNode(".//svc/sv/Node3D/ab");
        cansat = (G.Node3D)GetNode("./svc/sv/Node3D/rb/cansat");
        cam = (G.Camera3D)GetNode("./svc/sv/Node3D/cam");
        cr = (G.ColorRect)GetNode("./cr");
        vo = (G.MeshInstance3D)GetNode("./svc/sv/Node3D/visualObj");

        // setup
        ori[0] = new Quat(new Vec3(1,0.2f,0) , 0.0F);
        ori[1] = new Quat(new Vec3(1,0.2f,0) , 0.0F);
        //ori[0] = new Quat(new Vec3(0.0f,1.0f,0) , 4.0F);
        //ori[1] = new Quat(new Vec3(0.0f,1.0f,0) , 4.0F);
        // vectorized ori
        oriv[0] = new Vec3();
        oriv[1] = new Vec3();

        
        G.Transform3D transs = G.Transform3D.Identity;
        transs.Origin = new G.Vector3(camd,0,0);
        transs.Basis.X = new G.Vector3(0,1,0);
        transs.Basis.Y = new G.Vector3(0,0,1);
        transs.Basis.Z = new G.Vector3(1,0,0);
        cam.Transform = transs;

        transs.Origin = new G.Vector3(0,0,0);
        transs.Basis.X = (new Vec3(1,0,0).rotate(ori[0])).toGV();
        transs.Basis.Y = (new Vec3(0,1,0).rotate(ori[0])).toGV();
        transs.Basis.Z = (new Vec3(0,0,1).rotate(ori[0])).toGV();
        rb.Transform = transs;

        svc = (G.SubViewportContainer)GetNode("./svc");
        svc.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        svc.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));


        rb.Mass = cm;
        rb.Inertia = cmoi_p.toGV();
        rb.CenterOfMass = r.toGV();

        GD.Print($"constexpr float Ixx = {Ixx};");
        GD.Print($"constexpr float Ixy = {Ixy};");
        GD.Print($"constexpr float Ixz = {Ixz};");
        GD.Print($"constexpr float Iyx = {Ixy};");
        GD.Print($"constexpr float Iyy = {Iyy};");
        GD.Print($"constexpr float Iyz = {Iyz};");
        GD.Print($"constexpr float Izx = {Ixz};");
        GD.Print($"constexpr float Izy = {Iyz};");
        GD.Print($"constexpr float Izz = {Izz};");
        GD.Print($"constexpr float Wzz = {wmoi2};");

        //GD.Print($"constexpr float B_PMOI[3] = {{{cmoi.x}f,{cmoi.y}f,{cmoi.z}f}}; //x,y,z");
        //GD.Print($"constexpr float W_PMOI[2] = {{{wmoi1}f,{wmoi2}f}}; // x,z");

        GD.Print($"constexpr float TOR_MAX = {maxtorque}.0f;");
    }

    public override void _Input(Godot.InputEvent Event){
        if(Event is Godot.InputEventMouseButton mouseClickEvent){ //[InputEvent] is [InputEventMouseButton] [variable name] 方の一致不一致と型変換と代入と宣言を同時に行うscopeはたぶん_Inputのなか
            //click
            switch(mouseClickEvent.ButtonIndex){
                case Godot.MouseButton.Left:
                    Input.MBL = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Right:
                    Input.MBR = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.Middle:
                    Input.MBM = mouseClickEvent.Pressed;
                    break;
                case Godot.MouseButton.WheelUp:
                case Godot.MouseButton.WheelDown:
                    break;
            }
        } 
        if(Event is Godot.InputEventMouseMotion mouseMotionEvent){
            //move
            //if(Input.isMouseInside){
            Input.MMV[0] = mouseMotionEvent.Position;
                if(Input.MBR){
                    //回転操作
                            camAngl[0] -= sensit * 2 * FOV * (Input.MMV[0].X - Input.MMV[1].X)/svc.Size.X;
                        if(camAngl[0] <= -MathF.PI){
                            camAngl[0] += 2 * MathF.PI;
                        }else if( MathF.PI <= camAngl[0] ){
                            camAngl[0] -= 2 * MathF.PI;
                        }
                        //制限をかけるため 角度へらしてるときには-piの制限，ふやしてるときには+pi こうすれば制限外にでても復帰できる
                        if(Input.MMV[0].Y - Input.MMV[1].Y < 0){
                            if(-MathF.PI/2 < camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }else{
                            if(MathF.PI/2 > camAngl[1])
                                    camAngl[1] += sensit * 2 * FOV * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                        }
                            //camのTransformに適用
                            G.Transform3D transs = G.Transform3D.Identity;

                            GD.Print($"camangl ${camAngl[0]}, ${camAngl[1]}");

                            //camAngl[0] = (float)time * 10;
                            Quat q = new Quat(new Vec3(0,0,1), camAngl[0]);

                            q = Quat.concat(q,new Quat(new Vec3(0,-1,0) , camAngl[1]));
                            
                            transs.Origin = (new Vec3(camd,0,0).rotate(q)).toGV();

                            GD.Print($"origin {transs.Origin.X},{transs.Origin.Y},{transs.Origin.Z},");

                            transs.Basis.X = (new Vec3(0,1,0).rotate(q)).toGV();
                            transs.Basis.Y = (new Vec3(0,0,1).rotate(q)).toGV();
                            transs.Basis.Z = (new Vec3(1,0,0).rotate(q)).toGV();


                            cam.Transform = transs;
                }else if(Input.MBM){
                }
                if(Input.MBL){
                    // dragging
                    if((Input.MMV[0] - ppos - cr.Position - cr.Size/2).Length() < 10){
                        Input.isDragging = true;
                    }
                }else{
                    Input.isDragging = false;
                }
                
            Input.MMV[1] = Input.MMV[0];
            //}
        }
        if(Event is Godot.InputEventKey keyEvent){
            //key
            if(keyEvent.Pressed){
                //enter
                /*
                if(cmdBox.HasFocus() || tsN[0].HasFocus() || tsN[1].HasFocus() || fovN.HasFocus() || exposN.HasFocus()){
                    switch(keyEvent.Keycode){
                        case Godot.Key.Enter:
                            if(cmdBox.HasFocus()){
                                sendCmd();
                                cmdBox.AcceptEvent();
                            }
                            if(tsN[0].HasFocus()){
                                tsSet(0);
                            }
                            if(tsN[1].HasFocus()){
                                tsSet(1);
                            }
                            if(fovN.HasFocus()){
                                camN.Fov = float.Parse(fovN.Text);
                                FOV = toRadF(camN.Fov);
                            }
                            if(exposN.HasFocus()){

                            }
                            break;
                        case Godot.Key.Escape:
                            cmdBox.ReleaseFocus();
                            break;
                        default:
                            break;
                    }
                }else{
                */
                    switch(keyEvent.Keycode){
                        case Godot.Key.Space:
                            break;
                        case Godot.Key.S:
                            doControl = true;
                            break;
                        case Godot.Key.Q:
                            doControl = false;
                            break;
                        case Godot.Key.Key1:
                            target = Quat.concat(new Quat(new Vec3(0,0,1), MathF.PI/2), target);
                            break;
                        case Godot.Key.Key2:
                            break;
                        case Godot.Key.Key3:
                            break;
                        case Godot.Key.Key4:
                            break;
                        case Godot.Key.Key5:
                            break;
                        case Godot.Key.Key6:
                            break;
                        case Godot.Key.Key7:
                            break;
                        case Godot.Key.Key8:
                            break;
                        case Godot.Key.Key9:
                            break;
                        case Godot.Key.Key0:
                            break;
                        default:
                            break;
                    }
            }
        }
    }
    private void OnMouseEntered(){
        Input.isMouseInside = true;
    }
    private void OnMouseExited(){
        Input.isMouseInside = false;
    }
    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta) {
        time += delta;

            // instance → data  update
        pos.setFromGV(ab.Transform.Origin);
        ori[0].setFromGQ(new G.Quaternion(rb.Transform.Basis));
        oriv[0] = ori[0].toV();
        //avel = Vec3.divide(Vec3.add(oriv[0] , oriv[1].invert()) , (float)delta);
        avel.setFromGV(rb.AngularVelocity);
        avel = avel.rotate(ori[0].invert());

            //GD.Print($"ori {ori.toString()}");
        //GD.Print(rb.AngularVelocity.X);
        //avel.setFromGV(rb.AngularVelocity);
            //DebugDraw3D.DrawArrow(new G.Vector3(0,0,0), rb.AngularVelocity*20, G.Colors.Red, 1F, false);
            //DebugDraw3D.DrawCylinder(new G.Transform3D(G.Basis.Identity,rb.AngularVelocity*20));
            //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), rb.AngularVelocity*20, 2f, G.Colors.Red, 0.0f);
            //DebugDraw3D.DrawArrowRay(new G.Vector3(0,0,0), new G.Vector3(100,0,0), 80f, G.Colors.Lavender, 50f, true);
        //avel = avel.rotate(ori[0].invert());
            //GD.Print($"avel(local) : {(new Vec3(1000,0,0).rotate(ori.invert())).toString()}");



        /*
        double deltaT = 1;
        double startTime = 1;
        if(startTime < time && time < startTime+deltaT){
            doControl = true;

        }else if(startTime+deltaT < time && time < startTime+2*deltaT){
            doControl = false;
        }else{
            doControl = false;
        }
        */


        if(doControl){
            majiKeisandekiru3(delta);
        }else{
            torque.x = 0;
            torque.y = 0;
            torque.z = 0;
        }


            // local to global   apply torque(change energy of reaction wheel)
        rb.ApplyTorque(Vec3.add(
                    torque ,
                    Vec3.multiply( Vec3.cross( avel , wav ) , wmoi2 )
                    ).rotate(ori[0]).toGV());
        //rb.ApplyTorque(torque.rotate(ori[0]).toGV());


        ori[1] = ori[0];
        oriv[1] = oriv[0];

        /*


        // 1/2 で実行
        if(f2){
        }

        // for test
        //avel.multiply((1-ATTE*delta));

        //impulse = Vec3.multiply(force, delta);

        // Fdt = dp = mdv mdx/dt
        //Vec3 acc = Vec3.multiply(force , delta/m);
        //vel.add(acc);
        //vel.add(Vec3.multiply(impulse , 1/m));
        vel.add(Vec3.multiply(force , delta/m));
        pos.add(Vec3.multiply(vel, delta));

        double tdelta = delta;
        Vec3 aimpulse = Vec3.multiply(torque , tdelta);
        Vec3 tv = Vec3.add( force.invert() , new Vec3(0,0,GRAV*m ));
        for(int i=0; i<STEP; i++){

            // eular method  
            
            // torque * dt = r×Fdt
            //aimpulse = Vec3.add(aimpulse,Vec3.cross( r , tv.rotate(ori.invert())));
            aimpulse = Vec3.cross( r , tv.rotate(ori.invert()));

            //GD.Print($"torque : {torquetmp.toString()}");
                // calculate second derivative
                // coordinate G to L
            Vec3 avelL = avel.rotate(ori.invert());

            //GD.Print($"avelL : {avelL.toString()}");
                // from "eular's equation of motion" ( at primary moment axis coordinate )
            Vec3 aavel = new Vec3();
            aavel.x = (aimpulse.x + (pmoi.y - pmoi.z)*avelL.y*avelL.z) / pmoi.x ; 
            aavel.y = (aimpulse.y + (pmoi.z - pmoi.x)*avelL.z*avelL.x) / pmoi.y ; 
            aavel.z = (aimpulse.z + (pmoi.x - pmoi.y)*avelL.x*avelL.y) / pmoi.z ; 

            //GD.Print($"aavel : {aavel.toString()}");

                // coordinate trans. L2G
            aavel = aavel.rotate(ori);
            //aavel.multiply(tdelta);

                //
                // cal. first derivative
                //
            avel.add(Vec3.multiply(aavel,tdelta));
            //GD.Print($"avel : {avel.toString()}");

            //GD.Print($"pos : {pos.toString()}");


            //avel = new Vec3(0.05,0.05,0.00);



                // cal. zero d.
            ori = Quat.concat(new Quat(avel , avel.abs()*tdelta), ori);
            //GD.Print($"ori : {ori.toString()}");
            //GD.Print($"pos : {pos.toString()}");
            //GD.Print($"quat : {ori.toString()}");
            //GD.Print($"|quat| : {Math.Sqrt(Math.Pow(ori.w,2)+Math.Pow(ori.x,2)+Math.Pow(ori.y,2)+Math.Pow(ori.z,2))}");
        }

        //pos.x = (double)rpos[0].X*camd/(double)cr.Size.X;
        //pos.y = (double)rpos[0].Y*camd/(double)cr.Size.Y;

            // substitude 
        trans.Origin = pos.toGV();
        trans.Basis.X = (new Vec3(1,0,0).rotate(ori)).toGV();
        trans.Basis.Y = (new Vec3(0,1,0).rotate(ori)).toGV();
        trans.Basis.Z = (new Vec3(0,0,1).rotate(ori)).toGV();


        //trans.Origin = new G.Vector3(0,(float)time*100,0);
        //trans.Basis.X = new G.Vector3(1,0,0);
        //trans.Basis.Y = new G.Vector3(0,1,0);
        //trans.Basis.Z = new G.Vector3(0,0,1);

        //GD.Print($"basis x : {(new Vec3(1,0,0).rotate(ori)).toString()}");
        //GD.Print($"basis y : {(new Vec3(0,1,0).rotate(ori)).toString()}");
        //GD.Print($"basis z : {(new Vec3(0,0,1).rotate(ori)).toString()}");


        cansat.Transform = trans;
        //GD.Print($"{pos.x},{pos.y},{pos.z}");

        f2 = !f2;
        */
    }
    public override void _PhysicsProcess(double delta) {
            // moving 
        if(Input.isDragging){
            trans.Origin.X = camd*(Input.MMV[0].X - cr.Position.X - cr.Size.X/2)/cr.Size.X;
            trans.Origin.Y = camd*(-Input.MMV[0].Y + cr.Position.Y + cr.Size.Y/2)/cr.Size.Y;
            ab.Transform = trans;
        }
        //trans.Origin.Y = -camd + (float)time*1000;
        //    ab.Transform = trans;
    }
    private void majiKeisandekiru3(double delta){
        // bno can export  acceleration , orientation , gyro(= anglar velocity)


        //G.Transform3D transs = vo.Transform;
        //transs.Basis = new G.Basis(Quat.concat(new Quat(new Vec3(0,1,0) , tmp) , ori).toGQ());
        //vo.Transform = transs;


        // gain < 0にしてあるので，errorといいながらerrorではなくその反転
        Quat errorQ = Quat.concat(target.invert(), ori[0]);
        error = errorQ.toV();
        integral.add(error);

        GD.Print($"avel : {avel.toString()}");
        deltaV = Vec3.multiply(avel , Kd);
        deltaV.add(Vec3.multiply( error , Kp ));
        deltaV.add(Vec3.multiply( integral , Ki ));




        // deltaV to torque
        // Eular's equation of motion
        torque.x = cmoi.x*deltaV.x/(float)delta ; //(cmoi.z - cmoi.y)*avel.z*avel.y;
        torque.y = cmoi.y*deltaV.y/(float)delta ; //(cmoi.x - cmoi.z)*avel.x*avel.z;
        torque.z = cmoi.z*deltaV.z/(float)delta ; //(cmoi.y - cmoi.x)*avel.y*avel.x;
        //Vec3 gyro_correction = Vec3.multiply( Vec3.cross( avel , wav ) , wmoi2-wmoi1 );
        //torque.add(gyro_correction.invert());
        GD.Print($"ori : {ori[0].toString()}");
        GD.Print($"error : {errorQ.toString()}");
        GD.Print($"oriv : {oriv[0].toString()}");
        GD.Print($"deltaV : {deltaV.toString()}");
        GD.Print($"torque : {torque.toString()}");

        //    wav.add(maxwav/2);
        // true wav value area

        // 回転数が範囲外にならないようにする
        // 1.今加えられる各軸で最大のトルクを正と負でそれぞれ出す
        // 2.torqueをそれで正負クリップ
        // 3.maxtorqueでもクリップ
        // 4.

        Vec3 min = new Vec3(wmoi2*(-wav.x)/(float)delta,
                 wmoi2*(-wav.y)/(float)delta,
                 wmoi2*(-wav.z)/(float)delta);
        Vec3 max = new Vec3(wmoi2*(maxwav-wav.x)/(float)delta,
                 wmoi2*(maxwav-wav.y)/(float)delta,
                 wmoi2*(maxwav-wav.z)/(float)delta);
        if(torque.clip(min,max) == 0){
            GD.Print($"ここ！！！！！！！！！！！！！！！！！！！！${min.toString()} ${max.toString()}");
        }
        if(torque.clip(-maxtorque, maxtorque) == 0){
            GD.Print("ここ！！！！！！！！！！！！！！！！！！！！");
        }

        GD.Print($"torque' : {torque.toString()}");

        wav.add(Vec3.multiply(torque , (float)delta/wmoi2));

        //    wav.add(-maxwav/2);
        // to avoid saturation ( stabilize wav into normwav )
        torque.x += wmoi2 * (normwav-wav.x) *restoringFactor/(float)delta * 0.8F;
        torque.y += wmoi2 * (normwav-wav.y) *restoringFactor/(float)delta * 0.8F;
        torque.z += wmoi2 * (normwav-wav.z) *restoringFactor/(float)delta * 0.8F;
        wav.x += (normwav-wav.x) *restoringFactor;
        wav.y += (normwav-wav.y) *restoringFactor;
        wav.z += (normwav-wav.z) *restoringFactor;
         //if(maxwav <= wav.x || maxwav <= wav.y || maxwav <= wav.z || wav.x <= 0 || wav.y <= 0 || wav.z <= 0){
         //    GD.Print($" ${(normwav-wav.x) *restoringFactor} ${(normwav-wav.y) *restoringFactor} ${(normwav-wav.z) *restoringFactor}");
         //}
         //if( maxwav/2 < MathF.Abs(wav.x)){
         //    torque.x = 0;
         //}
         //if( maxwav/2 < MathF.Abs(wav.y)){
         //    torque.y = 0;
         //}
         //if( maxwav/2 < MathF.Abs(wav.z)){
         //    torque.z = 0;
         //}
        if(wav.x <= 0){
            wav.x = 0;
        }
        if(wav.y <= 0){
            wav.y = 0;
        }
        if(wav.z <= 0){
            wav.z = 0;
        }
        if(wav.x >= maxwav){
            wav.x = maxwav;
        }
        if(wav.y >= maxwav){
            wav.y = maxwav;
        }
        if(wav.z >= maxwav){
            wav.z = maxwav;
        }

        GD.Print($"wav : {wav.toString()}");

        GD.Print("\n");

        // for graph
        //red.Add(-torque.x*100/maxtorque);
        red.Add(-error.z*100);
        //green.Add(-torque.y*100/maxtorque);
        green.Add(-avel.z*100);
        blue.Add(-torque.z*100/maxtorque);
        black.Add(-wav.z);
        if( 100 < red.Count ){
            blue.RemoveAt(0);
            green.RemoveAt(0);
            red.RemoveAt(0);
            black.RemoveAt(0);
        }
        //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), torque.toGV() /20, 0.1f, G.Colors.Black, 0.0f);
        //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), torque.toGV() /20, 0.5f, G.Colors.Black, 0.0f);
        //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), torque.toGV() /20, 1f, G.Colors.Black, 0.0f);
        //DebugDraw3D.DrawCylinderAb(new G.Vector3(0,0,0), torque.toGV() /20, 1.5f, G.Colors.Black, 0.0f);
        //for(int i=0; i<10; i++){
        //    DebugDraw3D.DrawCylinderAb(rb.Transform.Origin, rb.Transform.Origin + torque.toGV() *100/maxtorque, (float)i/5, G.Colors.Black, 0.0f);
        //}
    }
}

/* memo for you
    - とりあえず制御の理論はできたんだじぇ
    - MOIのeigen valueどうやってだすじょ
    - ↑ MathNet.Numerics.LinearAlgebra.Factorization のevdでいけるっぽいんだじぇ
    - motorと機体のMOIも出せた
    - cilpも各次元での最小値とったらなんかよさそうなんだじぇ あとはモーター回転数について実験すれば""
    - setrsの最大値1024*0.8ぐらいがいいかも
    - primary MOI出して計算してるのに座標系戻し忘れてた，でもMOIの非対角成分これ相当小さいから無視してもよくね
    - ↑とりあえず無視すぶ
*/
