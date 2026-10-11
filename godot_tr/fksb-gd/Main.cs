using Godot;
using System;
using System.Buffers.Binary;
using System.Collections.Generic;
using System.IO;

public partial class Main : Control
{
    const double disappearingTime = 0.3; // msec

    const int INTERFACE = 1;
    const int VENDOR = 0x0403;
    const int M_PRODUCT = 0x6001;
    const int R_PRODUCT = 0x6015;
    const int BAUDRATE = 115200;
    const int normalDatSize = 17;
    const int saveDatSize = 9;
    const int flashDatSize = 30;
    const int flashMax = 0x07FFFF;
    const string recordDir = "/home/legrs/fksb/record/";

    const float grayValue = 0.92F;

    // ftdi
    int len = 0;

    //byte[] dat = new byte[datSize];

        // Godot classes
    public static Color red = new Color(10F,0.0f,0.0f);
    public static Color green = new Color(0.0F,10f,0.0f);
    public static Color blue = new Color(0.0F,0.0f,10f);
    public static Color gray = new Color(grayValue,grayValue,grayValue);
    public static Node monostick;
    public static Node r3;
    public static Panel piBno;
    public static Panel piMemory;

    public static Panel piRunning;
    public static Panel piWriting;
    public static Panel piLaunched;
    public static Panel piSeparated;
    public static Panel piLanding;

    public static Panel piControlling;
    public static Panel piStarted;
    public static Panel piStopped;
    public static Panel piReceive;

    public static Panel piMono;
    public static Panel piR3;
    public static Panel piReversed;

    public static RichTextLabel modeText;

    public static ColorRect graphArea;
    public static ColorRect svcc;
    public static RichTextLabel tl;
    public static RichTextLabel flashRaw;
    public static RichTextLabel flashIndicator;
    public static RichTextLabel raw;
    public static RichTextLabel cmd;
    public static RichTextLabel clock;
    public static SubViewportContainer svc;
    public static SubViewport sv;

    public static Node3D cansat;
    public static Camera3D cam;
    public static TabContainer tc;

    public static Button extract;
    public static ProgressBar progress;

    Transform3D trans = Transform3D.Identity;

        // display
    double receivedTime;
    public struct Input{
        public static bool MBL = false;//mouse light button
        public static bool MBR = false;// \\   right
        public static bool MBM = false;// \\   middle
        public static float[] MWV = new float[2]{0,0}; //wheel
        public static Godot.Vector2[] MMV = new Vector2[2]; //mouse
        public static Godot.Vector2 camDiff = new Godot.Vector2(242,145);
        public static float[] scale = new float[2]{1,0.000001F};
        public static bool isMouseInside = false;
        public static bool isDragging = false;
    }

    public static float[] camAngl = new float[2]{0,0};
    float sensit = 3F;
    public static float camd = 800F;
    string pathDate;


        // dat
    public static double time;
    public static List<float>[] graphDat = new List<float>[4];
    public static Color[] graphColor = new Color[4]{blue , green , red , new Color(0,0,0)};
    public const int graphDatLen = 400;
    public const float arrowAngl = 9;
    public const float arrowLength = 15;

    public static float gain = 0.1f;
    public static float kp = 0.6f;
    public static float kd = 0.2f;
    public static float ki = 0.0f;

    public static Quat ori = new Quat();
    public static Vec3 avel = new Vec3();
    public static Vec3 acce = new Vec3();
    public static Vec3 mag = new Vec3();
    public static Vec3 mpv = new Vec3();
    public static short[] mp = new short[3]{0,0,0};
    public static int mode;
    public static float temp = 0;
    public static byte flag1 = 0;
    public static byte flag2 = 0;
    public static byte index = 0;

    public static int datIndex = 0;

    public static bool isReversed = false;

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
            if(Input.isMouseInside){
                Input.MMV[0] = mouseMotionEvent.Position;
                    if(Input.MBR){
                        //回転操作
                                camAngl[0] -= sensit * (Input.MMV[0].X - Input.MMV[1].X)/svc.Size.X;
                            if(camAngl[0] <= -MathF.PI){
                                camAngl[0] += 2 * MathF.PI;
                            }else if( MathF.PI <= camAngl[0] ){
                                camAngl[0] -= 2 * MathF.PI;
                            }
                            //制限をかけるため 角度へらしてるときには-piの制限，ふやしてるときには+pi こうすれば制限外にでても復帰できる
                            if(Input.MMV[0].Y - Input.MMV[1].Y < 0){
                                if(-MathF.PI/2 < camAngl[1])
                                        camAngl[1] += sensit * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                            }else{
                                if(MathF.PI/2 > camAngl[1])
                                        camAngl[1] += sensit * (Input.MMV[0].Y - Input.MMV[1].Y)/svc.Size.Y;
                            }
                                //camのTransformに適用
                                Transform3D transs = Transform3D.Identity;

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
                        Input.isDragging = true;
                    }else{
                        Input.isDragging = false;
                    }
                    
                Input.MMV[1] = Input.MMV[0];
            }
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
                    char keyc = (char)keyEvent.Unicode;
                    if(keyc > 0){
                        if(isReversed){
                            GD.Print($"send {keyc} : {r3.Call("write" , (int)keyc)}");
                        }else{
                            GD.Print($"send {keyc} : {monostick.Call("write" , (int)keyc)}");

                        }
                    }

                    DateTime now = DateTime.Now;
                    
                    cmd.SetText($"{cmd.Text}\n{now.ToString("HH:mm:ss")} - {keyc} ");

                    switch(keyEvent.Keycode){
                        case Godot.Key.Space:
                            break;
                        case Godot.Key.S:
                            monostick.Call("init" , INTERFACE,VENDOR,M_PRODUCT,BAUDRATE);
                            r3.Call("init" , INTERFACE,VENDOR,R_PRODUCT,BAUDRATE);
                            break;
                        case Godot.Key.R:
                            isReversed = !isReversed;
                            if(isReversed){
                                piReversed.Modulate = red;
                            }else{
                                piReversed.Modulate = gray;
                            }

                            break;
                        case Godot.Key.I:
                            kp += 0.1f;
                            break;
                        case Godot.Key.K:
                            kp -= 0.1f;
                            break;
                        case Godot.Key.L:
                            kd += 0.05f;
                            break;
                        case Godot.Key.J:
                            kd -= 0.05f;
                            break;
                        case Godot.Key.T:
                            gain += 0.1f;
                            break;
                        case Godot.Key.G:
                            gain -= 0.1f;
                            break;
                        case Godot.Key.Y:
                            ki += 0.1f;
                            break;
                        case Godot.Key.H:
                            ki -= 0.1f;
                            break;
                        default:
                            break;
                    }
                    GD.Print($"kp : {kp}");
                    GD.Print($"kd : {kd}");
                    GD.Print($"ki : {ki}");
                    GD.Print($"gain : {gain}");
                //}
            }
        }
    }
    private void OnMouseEntered(){
        Input.isMouseInside = true;
    }
    private void OnMouseExited(){
        Input.isMouseInside = false;
    }
    private bool isFlag(byte flag, int index){
        return (flag & (1<<index)) != 0;
    }
    // $"{dat}"だと"System.Byte[]"という文字列が書かれてしまうので，byte列をそのまま追記する
    private static void appendBytes(string path, byte[] dat){
        using(FileStream fs = new FileStream(path, FileMode.Append, FileAccess.Write)){
            fs.Write(dat, 0, dat.Length);
        }
    }
    // ftdiのreadは届いている分しか返さないので，size byteそろうまで(数回まで)読みたす
    private byte[] readFull(Node dev, int size){
        List<byte> buf = new List<byte>();
        for(int retry=0; buf.Count < size && retry < 20; retry++){
            byte[] d = (byte[])dev.Call("read" , size - buf.Count);
            if(d.Length == 0){
                System.Threading.Thread.Sleep(1);
            }
            buf.AddRange(d);
        }
        return buf.ToArray();
    }
    public async void Extract(){
        if((int)r3.Call("isOK") == 1){
            datIndex = 0;
            r3.Call("write" , (int)'s');
            
            int len = 100;
            while(len != 0){
                byte[] dat = (byte[])r3.Call("read" , flashDatSize);
                len = dat.Length;
            }
            File.WriteAllText($"{recordDir}flash/{pathDate}_decoded.csv" , "index,flag1,flag2,ori,avel,acce,mp,temp ( Big-Endian )\n");

            flashIndicator.SetText($"抽出中...");
            for(int adr = 0; adr*flashDatSize<flashMax; adr++){
                r3.Call("write" , (int)'r');
                byte[] dat = readFull(r3, flashDatSize);
                if(dat.Length == flashDatSize){
                    index = dat[0];
                    flag1 = dat[1];
                    flag2 = dat[2];
                    acce.setFromByte(dat.AsSpan(3));
                    acce.divide(100);
                    mp[0] = (short)((dat[9]<<8) | (dat[10]));
                    mp[1] = (short)((dat[11]<<8) | (dat[12]));
                    mp[2] = (short)((dat[13]<<8) | (dat[14]));
                    mpv.x = (float)mp[0];
                    mpv.y = (float)mp[1];
                    mpv.z = (float)mp[2];
                    temp = (int)dat[15];
                    avel.setFromByte(dat.AsSpan(16));
                    avel.divide(900);
                    ori.setFromByte(dat.AsSpan(22));
                    File.AppendAllText($"{recordDir}flash/{pathDate}_decoded.csv" , $"{index},{Convert.ToString(flag1, 2).PadLeft(8, '0')},{Convert.ToString(flag2, 2).PadLeft(8, '0')},{ori.toString()},{avel.toString()},{acce.toString()},{mpv.toString()},{temp}\n");
                    appendBytes($"{recordDir}flash/{pathDate}_raw.bin" , dat);
                    if(100 <=datIndex){
                        //100こえたら重いからリセット
                        flashRaw.SetText($"{BitConverter.ToString(dat)}");
                        datIndex = 0;
                    }else{
                        flashRaw.SetText($"{flashRaw.Text}\n{BitConverter.ToString(dat)}");
                    }
                    progress.Value = (float)adr*flashDatSize/(float)flashMax;

                    if(adr%3 == 0){
                        await ToSignal(GetTree(), "process_frame");
                    }
                    datIndex++;
                }

            }
            GD.Print("extract done");
            flashIndicator.SetText($"抽出完了.");
        }else{
            flashIndicator.SetText($"Tweliteとの通信に失敗.");
        }


    }

    // Called when the node enters the scene tree for the first time.
    public override void _Ready()
    {
        for(int i=0; i<4; i++){
            graphDat[i] = new List<float>();
        }
        monostick = GetNode("./monostick");
        r3 = GetNode("./r3");
        piReceive =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piReceive/point");
        piBno =             (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piBno/point");
        piMemory =          (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piMemory/point");
        piRunning =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piRunning/point");
        piWriting =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piWriting/point");
        piLaunched =        (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piLaunched/point");
        piSeparated =       (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piSeparated/point");
        piLanding =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piLanding/point");
        piControlling =     (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piControlling/point");
        piStarted =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piStarted/point");
        piStopped =         (Panel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/piStopped/point");
        piMono =         (Panel)GetNode("./piMono/point");
        piR3 =         (Panel)GetNode("./piR3/point");
        piReversed =         (Panel)GetNode("./piReversed/point");
        modeText =         (RichTextLabel)GetNode("./tc/realtime/indicator/hsc2/vc/vc2/mode");
        tl =        (RichTextLabel)GetNode("./tc/realtime/indicator/hsc2/vc/txt");
        raw =       (RichTextLabel)GetNode("./tc/realtime/indicator/hsc2/vc/raw");
        clock =       (RichTextLabel)GetNode("./clock");
        cmd =        (RichTextLabel)GetNode("./tc/realtime/indicator/hsc2/vc2/txt2");
        graphArea =     (ColorRect)GetNode("./tc/realtime/vsc/cr1");
        svcc      =     (ColorRect)GetNode("./tc/realtime/vsc/cr2");
        tc      =     (TabContainer)GetNode("./tc");
        svc =(SubViewportContainer)GetNode("./tc/realtime/vsc/cr2/svc");
        sv =          (SubViewport)GetNode("./tc/realtime/vsc/cr2/svc/sv");
        cansat =           (Node3D)GetNode("./tc/realtime/vsc/cr2/svc/sv/Node3D/fksb");
        cam =            (Camera3D)GetNode("./tc/realtime/vsc/cr2/svc/sv/Node3D/cam");

        flashRaw =   (RichTextLabel)GetNode("./tc/flash/hc/vc/txt3");
        flashIndicator=(RichTextLabel)GetNode("./tc/flash/hc/vc/txt2");
        extract =   (Button)GetNode("./tc/flash/hc/vc/Button");
        progress =   (ProgressBar)GetNode("./tc/flash/hc/vc/progress");

        svc.Connect("mouse_entered", new Godot.Callable(this, nameof(OnMouseEntered)));
        svc.Connect("mouse_exited", new Godot.Callable(this, nameof(OnMouseExited)));

        extract.Pressed += Extract;

        cansat.Transform = trans;

        monostick.Call("init" , INTERFACE,VENDOR,M_PRODUCT,BAUDRATE);
        r3.Call("init" , INTERFACE,VENDOR,R_PRODUCT,BAUDRATE);

        GD.Print("\n");

        DateTime now = DateTime.Now;
        pathDate = now.ToString("yyMMdd-HHmm");



        File.WriteAllText($"{recordDir}realtime/{pathDate}_decoded.csv" , $"FksB data obtained with FksB support program @{now.ToString("yyyy/MM/dd HHmm ss")}\n");
        File.AppendAllText($"{recordDir}realtime/{pathDate}_decoded.csv" , $"time,index,flag1,flag2,ori,mp ( Big-Endian )\n");

        File.WriteAllText($"{recordDir}realtime/{pathDate}_raw.bin" , $"FksB data obtained with FksB support program @{now.ToString("yyyy/MM/dd HHmm ss")}\n");
    }

    // Called every frame. 'delta' is the elapsed time since the previous frame.
    public override void _Process(double delta)
    {
        DateTime now = DateTime.Now;
        clock.SetText($"JST {now.ToString("yyyy/MM/dd HH:mm:ss")}");

        if((int)monostick.Call("isOK") == 1){
            piMono.Modulate = red;
        }else{
            piMono.Modulate = gray;
        }
        if((int)r3.Call("isOK") == 1){
            piR3.Modulate = red;
        }else{
            piR3.Modulate = gray;
        }

        // realtime telemetry
        if(tc.CurrentTab == 0){
            bool isok = false;
            if(isReversed){
                isok = (int)r3.Call("isOK") == 1;
            }else{
                isok = (int)monostick.Call("isOK") == 1;
            }
            if(isok){
                time += delta;
                byte[] dat;
                if(isReversed){
                    dat = (byte[])r3.Call("read" , normalDatSize);
                }else{
                    dat = (byte[])monostick.Call("read" , normalDatSize);
                }
                if(dat.Length != 0){
                    //GD.Print(dat.Length);
                }
                if(3 < dat.Length){ //2はゴミの場合がある
                    //GD.Print($"{len} :  {System.Text.Encoding.ASCII.GetString(dat)}");
                    piReceive.Modulate = red;
                    receivedTime = time;

                    index = dat[0];
                    flag1 = dat[1];
                    flag2 = dat[2];

                    raw.SetText($" {BitConverter.ToString(dat)}");
                    //GD.Print($"flag2 {Convert.ToString(flag1, 2).PadLeft(8, '0')}");
                    mode = (flag1 & 0b00000011) + 1;
                    modeText.SetText($"[right]モード : {mode}");
                    if(isFlag(flag1,3)){
                        piStarted.Modulate = green;
                    }else{
                        piStarted.Modulate = gray;
                    }
                    if(isFlag(flag1,4)){
                        piStopped.Modulate = green;
                    }else{
                        piStopped.Modulate = gray;
                    }
                    if(isFlag(flag1,5)){
                        piBno.Modulate = blue;
                        if(isFlag(flag1,7)){
                            piBno.Modulate = green;
                        }else{
                            piBno.Modulate = blue;
                        }
                    }else{
                        piBno.Modulate = gray;
                        piBno.Modulate = gray;
                    }
                    if(isFlag(flag1,6)){
                        piMemory.Modulate = green;
                    }else{
                        piMemory.Modulate = gray;
                    }

                    if(mode == 1 && isFlag(flag1, 2)){
                        piLanding.Modulate = green;
                    }else{
                        piLanding.Modulate = gray;
                    }
                    if(1 <= mode){
                        piRunning.Modulate = green;
                    }else{
                        //ありえないけど
                        piRunning.Modulate = gray;
                    }
                    if(2 <= mode){
                        piWriting.Modulate = green;
                    }else{
                        piWriting.Modulate = gray;
                    }
                    if(3 <= mode){
                        piLaunched.Modulate = green;
                        if(isFlag(flag1, 2)){
                            piSeparated.Modulate = green;
                        }else{
                            piSeparated.Modulate = gray;
                        }
                    }else{
                        // separatedはもどさない
                        piLaunched.Modulate = gray;
                    }
                    if(4 == mode){
                        piControlling.Modulate = green;
                    }else{
                        piControlling.Modulate = gray;
                    }
                }else{
                    if(disappearingTime < time - receivedTime){
                        piReceive.Modulate = gray;
                    }else{
                        float fac = (float)((time-receivedTime)/disappearingTime);
                        piReceive.Modulate = new Color(10F - (10-grayValue)*(1-fac) , grayValue*fac, grayValue*fac);
                        //GD.Print((float)((time-receivedTime)/disappearingTime));
                    }
                }
                if(dat.Length == saveDatSize){
                    acce.setFromByte(dat.AsSpan(3));
                    acce.divide(100);
                    // unit of acce is G
                    tl.SetText($"\nindex : {index}\nAcc : ( {acce.toString()} )\n");
                }
                if(dat.Length == normalDatSize){
                    ori.setFromByte(dat.AsSpan(3));

                    mp[0] = (short)((dat[11]<<8) | (dat[12]));
                    mp[1] = (short)((dat[13]<<8) | (dat[14]));
                    mp[2] = (short)((dat[15]<<8) | (dat[16]));
                    mpv.x = (float)mp[0];
                    mpv.y = (float)mp[1];
                    mpv.z = (float)mp[2];
                    // 今回のpacketの値をdecodeしてから記録する (前は1つ前のpacketのori,mpが書かれていた)
                    File.AppendAllText($"{recordDir}realtime/{pathDate}_decoded.csv" , $"{Time.GetUnixTimeFromSystem()},{index},{Convert.ToString(flag1, 2).PadLeft(8, '0')},{Convert.ToString(flag2, 2).PadLeft(8, '0')},{ori.toString()},{mpv.toString()}\n");
                    appendBytes($"{recordDir}realtime/{pathDate}_raw.bin" , dat);
                    trans.Origin = new Vector3(0,0,0);
                    trans.Basis.X = (new Vec3(1,0,0)).rotate(ori).toGV();
                    trans.Basis.Y = (new Vec3(0,1,0)).rotate(ori).toGV();
                    trans.Basis.Z = (new Vec3(0,0,1)).rotate(ori).toGV();
                    cansat.Transform = trans;
                    tl.SetText($"\nindex : {index}\nOri : ( {ori.toString()} )\nmp : ( {mpv.toString()} )\ntemp : {temp}");
                }

            }else{
            }
            // extract flash data
        }else if(tc.CurrentTab == 1){
        }

    }

}
