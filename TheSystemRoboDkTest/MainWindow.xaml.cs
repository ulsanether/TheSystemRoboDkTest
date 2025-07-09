using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;

using RoboDk.API;


namespace TheSystemRoboDkTest;

/// <summary>
/// Interaction logic for MainWindow.xaml
/// </summary>
public partial class MainWindow : Window
{
    [DllImport("user32.dll")]
    private static extern IntPtr SetParent(IntPtr hWndChild, IntPtr hWndNewParent);

    [DllImport("user32.dll", SetLastError = true)]
    private static extern bool MoveWindow(IntPtr hWnd, int X, int Y, int nWidth, int nHeight, bool bRepaint);

    public RoboDK RDK = null;
    RoboDK.Item ROBOT = null;
    const bool MOVE_BLOCKING = false;


    private int StepMin = 0;
    private int StepMax = 2000;
    private int StepIncrement = 5;

    public MainWindow()
    {
        InitializeComponent();

        RoboDK rdk = new RoboDK();
    }

  

    public void CloseAllStations()
    {
        // Get all the RoboDK stations available
        RoboDK.Item[] all_stations = RDK.getItemList(RoboDK.ITEM_TYPE_STATION);

        if (all_stations == null || all_stations.Length == 0)
        {
            notifybar.Text = "No stations to close.";
            return;
        }

        foreach (RoboDK.Item station in all_stations)
        {
            try
            {
                notifybar.Text = "Closing " + station.Name();
                // 이 스테이션을 저장하지 않고 닫음
                station.Delete();
            }
            catch (Exception ex)
            {
                notifybar.Text = $"Failed to close station: {station.Name()} - {ex.Message}";
            }
        }
    }


    private void btnSelectRobot_Click(object sender, RoutedEventArgs e)
    {
        SelectRobot();
    }


    public void SelectRobot()
    {
        notifybar.Text = "Selecting robot...";
        if (!Check_RDK())
        {
            ROBOT = null;
            return;
        }
        ROBOT = RDK.ItemUserPick("Select a robot", RoboDK.ITEM_TYPE_ROBOT); // select robot among available robots
                                                                            //ROBOT = RL.getItem("ABB IRB120", ITEM_TYPE_ROBOT); // select by name
                                                                            //ITEM = RL.ItemUserPick("Select an item"); // Select any item in the station
        if (ROBOT.Valid())
        {
            ROBOT.NewLink(); // This will create a new communication link (another instance of the RoboDK API), this is useful if we are moving 2 robots at the same time.                
            notifybar.Text = "Using robot: " + ROBOT.Name();
        }
        else
        {
            notifybar.Text = "Robot not available. Load a file first";
        }
    }



    private void rad_RunMode_Simulation_CheckedChanged(object sender, RoutedEventArgs e)
    {
        // 라디오버튼이 체크 해제된 경우 무시
        if (sender is System.Windows.Controls.RadioButton rb && rb.IsChecked != true)
            return;

        // RoboDK 연결 확인
        if (!Check_RDK())
            return;

        // RoboDK 창을 일반 모드로 전환
        SetParent(RDK.GetWindowHandle(), IntPtr.Zero);

        RDK.setWindowState(RoboDK.WINDOWSTATE_NORMAL);      // 일반 모드
        RDK.setWindowState(RoboDK.WINDOWSTATE_MAXIMIZED);   // 최대화

        // WPF에서는 MinHeight/MinWidth 사용
        this.Height = this.MinHeight;
        this.Width = this.MinWidth;

        // WPF에서는 BringToFront가 없으므로, Activate() 사용
        this.Activate();

        // 상태 표시
        notifybar.Text = "Simulation 모드로 전환됨";
    }

    private void rad_RunMode_Program_CheckedChanged(object sender, RoutedEventArgs e)
    {
        // 라디오버튼이 체크 해제된 경우 무시
        if (rad_RunMode_Program.IsChecked != true) return;

        btnOLPdone.IsEnabled = true;
        if (!Check_ROBOT()) return;

        // 안전을 위해 로봇 연결 해제
        ROBOT.Finish();

        // 오프라인 프로그래밍(프로그램 생성) 모드로 설정
        RDK.setRunMode(RoboDK.RUNMODE_MAKE_ROBOTPROG);

        // 프로그램 이름, 폴더, 포스트프로세서 지정(필요시)
        RDK.ProgramStart("NewProgram");
    }

    private void rad_RunMode_Online_CheckedChanged(object sender, RoutedEventArgs e)
    {
        // 라디오버튼이 체크 해제된 경우 무시
        if (rad_RunMode_Online.IsChecked != true) return;

        btnOLPdone.IsEnabled = false;
        // RoboDK 연결 및 로봇 체크
        if (!Check_ROBOT()) return;

        // 이전 프로그램 생성 중지
        RDK.Finish();

        // 실제 로봇에 연결 시도
        if (ROBOT.Connect())
        {
            // 실제 로봇 실행 모드로 설정
            RDK.setRunMode(RoboDK.RUNMODE_RUN_ROBOT);
        }
        else
        {
            notifybar.Text = "Can't connect to the robot. Check connection and parameters.";
            rad_RunMode_Simulation.IsChecked = true;
        }
    }

    private void rad_RoboDK_show_CheckedChanged(object sender, RoutedEventArgs e)
    {
        // 라디오버튼이 체크 해제된 경우 무시
        if (rad_RoboDK_show.IsChecked != true) return;

        // RoboDK 연결 확인
        if (!Check_RDK()) return;

        // 패널에서 분리 (WPF에서는 일반적으로 필요 없음)
        // SetParent(RDK.GetWindowHandle(), IntPtr.Zero);

        // RoboDK 창을 일반/최대화 상태로 전환
        RDK.setWindowState(RoboDK.WINDOWSTATE_NORMAL);
        RDK.setWindowState(RoboDK.WINDOWSTATE_MAXIMIZED);

        // WPF 창 최소 크기로 조정
        this.Width = this.MinWidth;
        this.Height = this.MinHeight;

        // RoboDK 창을 앞으로 가져오기
        RDK.ShowRoboDK();
    }

    private void rad_RoboDK_hide_CheckedChanged(object sender, RoutedEventArgs e)
    {
        if (rad_RoboDK_hide.IsChecked != true) return;
        if (!Check_RDK()) return;

        RDK.setWindowState(RoboDK.WINDOWSTATE_HIDDEN);
        // RDK.HideRoboDK();

        // WPF 창 최소 크기로 조정
        this.Width = this.MinWidth;
        this.Height = this.MinHeight;
    }



    private void Window_Closing(object sender, CancelEventArgs e)
    {
        if (_hWnd != IntPtr.Zero)
        {
            SetParent(_hWnd, IntPtr.Zero);
            RDK.CloseRoboDK();
            RDK = null;
            _hWnd = IntPtr.Zero;
        }
    }


    private void rad_RoboDK_Integrated_CheckedChanged(object sender, RoutedEventArgs e)
    {
        if (rad_RoboDK_Integrated.IsChecked != true) return;
        if (!Check_RDK()) return;

        IntPtr rdkHandle = RDK.GetWindowHandle();
        if (rdkHandle == IntPtr.Zero)
        {
            notifybar.Text = "RoboDK 메인 창을 찾을 수 없습니다. RoboDK가 실행 중인지 확인하세요.";
            return;
        }

        // WPF의 MainWindow 핸들 얻기
        IntPtr mainWindowHandle = new System.Windows.Interop.WindowInteropHelper(this).Handle;

        // RoboDK 창을 MainWindow에 임베드
        SetParent(rdkHandle, mainWindowHandle);

        // border1의 위치와 크기 계산
        var relativePoint = border1.TransformToAncestor(this).Transform(new System.Windows.Point(0, 0));
        int x = (int)relativePoint.X;
        int y = (int)relativePoint.Y;
        int width = (int)border1.ActualWidth;
        int height = (int)border1.ActualHeight;

        // RoboDK 창을 border1 위치/크기에 맞게 이동 및 리사이즈
        MoveWindow(rdkHandle, x, y, width, height, true);

        // RoboDK 창을 시네마/전체화면 모드로 전환
        RDK.setWindowState(RoboDK.WINDOWSTATE_CINEMA);
        //RDK.setWindowState(RoboDK.WINDOWSTATE_FULLSCREEN);

        this.Height = 700;
    }



    private IntPtr _hWnd;


    private void border1_SizeChanged(object sender, SizeChangedEventArgs e)
    {
        if (_hWnd != IntPtr.Zero)
        {
            MoveWindow(_hWnd, (int)border1.Margin.Left, (int)border1.Margin.Top, (int)border1.ActualWidth, (int)border1.ActualHeight, true);
        }
    }


    private void btnMoveRobotHome_Click(object sender, RoutedEventArgs e)
    {
        if (!Check_ROBOT()) { return; }

        double[] joints_home = ROBOT.JointsHome();

        ROBOT.MoveJ(joints_home);
    }
    private void btnRunTestProgram_Click(object sender, EventArgs e)
    {


        if (!Check_ROBOT()) { return; }


        int n_sides = 6;

        Mat pose_ref = ROBOT.Pose();

        // Set the simulation speed (ratio = real time / simulated time)
        // 1 second of the simulator equals 5 second in real time
        RDK.setSimulationSpeed(5);

        try
        {
            // retrieve the reference frame and the tool frame (TCP)
            Mat frame = ROBOT.PoseFrame();
            Mat tool = ROBOT.PoseTool();
            int runmode = RDK.RunMode(); // retrieve the run mode 

            // Program start
            ROBOT.MoveJ(pose_ref);
            ROBOT.setPoseFrame(frame);  // set the reference frame
            ROBOT.setPoseTool(tool);    // set the tool frame: important for Online Programming
            ROBOT.setSpeed(100);        // Set Speed to 100 mm/s
            ROBOT.setZoneData(5);       // set the rounding instruction (C_DIS & APO_DIS / CNT / ZoneData / Blend Radius / ...)
            ROBOT.RunInstruction("CallOnStart", RoboDK.INSTRUCTION_CALL_PROGRAM);
            for (int i = 0; i <= n_sides; i++)
            {
                double angle = ((double)i / n_sides) * 2.0 * Math.PI;

                // calculate the next position
                Mat pose_i = pose_ref * Mat.rotz(angle) * Mat.transl(100, 0, 0) * Mat.rotz(-angle);

                // Add an instruction (comment)
                ROBOT.RunInstruction("Moving to point " + i.ToString(), RoboDK.INSTRUCTION_COMMENT);
                double[] xyzwpr = pose_i.ToXYZRPW(); // read the target as XYZWPR
                ROBOT.MoveL(pose_i);
            }
            ROBOT.RunInstruction("CallOnStart", RoboDK.INSTRUCTION_CALL_PROGRAM);
            ROBOT.MoveL(pose_ref);
        }
        catch (RoboDK.RDKException rdkex)
        {
            notifybar.Text = "Failed to complete the movement: " + rdkex.Message;
        }

        return;



        RDK.EventsListen();
        RDK.EventsLoop();
        return;


        // API communication speed tests
        var stopwatch = new Stopwatch();
        int ntests = 1000;

        stopwatch.Reset();
        stopwatch.Start();
        for (var i = 0; i < ntests; i++)
        {
            var robot_name = ROBOT.Name();
        }
        stopwatch.Stop();
        Console.WriteLine($"Calling .Name() took {stopwatch.ElapsedMilliseconds * 1000 / ntests} micro seconds on average");

        stopwatch.Reset();
        stopwatch.Start();
        for (var i = 0; i < ntests; i++)
        {
            var joints = ROBOT.Joints();
        }
        stopwatch.Stop();
        Console.WriteLine($"Calling .Joints() took {stopwatch.ElapsedMilliseconds * 1000 / ntests} micro seconds on average");
        return;



        RDK.EventsListen();
        RDK.EventsLoop();


        RDK.WaitForEvent(out int evt, out RoboDK.Item itm);
        RDK.SampleRoboDkEvent(evt, itm);
        return;

        //--------------------------------------------------
        // Other tests used for debugging...

        //RDK.SetInteractiveMode(RoboDK.SELECT_MOVE, RoboDK.DISPLAY_REF_TX | RoboDK.DISPLAY_REF_TY | RoboDK.DISPLAY_REF_PXY | RoboDK.DISPLAY_REF_RZ, new List<RoboDK.Item>() { ROBOT }, new List<int>() { RoboDK.DISPLAY_REF_ALL });
        //return;


        RoboDK.Item[] references = RDK.getItemList(RoboDK.ITEM_TYPE_FRAME);
        UInt64[] cam_list = new UInt64[references.Length];
        for (int i = 0; i < references.Length; i++)
        {
            cam_list[i] = RDK.Cam2D_Add(references[i], "FOCAL_LENGHT=6 FOV=32 FAR_LENGHT=1000 SIZE=640x480");
        }

        System.Threading.Thread.Sleep(2000);

        for (int i = 0; i < references.Length; i++)
        {
            RDK.Cam2D_SetParams(references[i].Name(), cam_list[i]);
        }

        System.Threading.Thread.Sleep(2000);

        // close all cameras
        RDK.Cam2D_Close();

        return;

        // Example to change the robot parameters (DHM parameters as defined by Craig 1986)
        // for joints 1 to 6, index i changes from 0 to 5:
        // dhm[i][] = [alpha, a, theta, d];


        // first point
        double[] p1 = { 0, 0, 0 };
        double[] p2 = { 1000, 0, 0 };
        Mat reference = Mat.transl(0, 0, 100);
        double[] p_collision = new double[3]; // this can be null if we don't need the collision point

        RoboDK.Item item = RDK.Collision_Line(p1, p2, reference, p_collision);

        string name;
        if (item.Valid())
        {
            name = item.Name();
        }
        else
        {
            // item not valid
        }

        return;




        //-------------------------------------------------------------
        // Test forward/inverse kinematics calculation for multiple robots
        var station = RDK.AddStation("Speed Tests");

        string robot_file = RDK.getParam("PATH_LIBRARY") + "/KUKA-KR-210-R2700.robot";
        int n_robots = 10;
        int n_tests = 100;
        List<RoboDK.Item> robot_list = new List<RoboDK.Item>();
        List<double[]> joints_list = new List<double[]>();

        for (int i = 0; i < n_robots; i++)
        {
            var robot = RDK.AddFile(robot_file);
            var joints = new double[] { i * 5, -90, 90, 0, 90, 0 };
            robot.setJoints(joints);
            robot_list.Add(robot);
            joints_list.Add(joints);
        }



        double time_average_ms = 0;

        for (int t = 0; t < n_tests; t++)
        {
            var t1 = DateTime.Now;

            // Bulk calculation (new): you can provide a list of robots: 2.5 ms for 10 robots on avg
            var pose_solutions = RDK.SolveFK(robot_list, joints_list);
            var joint_solutions = RDK.SolveIK(robot_list, pose_solutions);
            var joints_solutions2 = RDK.SolveIK(robot_list, pose_solutions, joints_list);
            var jnts = RDK.SolveIK_All(robot_list, pose_solutions);
            var cnfigs = RDK.JointsConfig(robot_list, joints_list);


            /*
            // Individual calculation (typical operation): 5.5 ms for 10 robots on avg
            for (int i = 0; i < n_robots; i++)
            {
                Mat pose = robot_list[i].SolveFK(joints_list[i]);
                var solution = robot_list[i].SolveIK(pose); // pose_solutions[i]);                    
            }
            */

            var t2 = DateTime.Now;
            var elapsed_ms = t2.Subtract(t1).TotalMilliseconds;
            time_average_ms = time_average_ms + elapsed_ms;
            Console.WriteLine("Forward/inverse kinematics Calculated in (ms)" + elapsed_ms.ToString());
        }
        time_average_ms = time_average_ms / n_tests;
        Console.WriteLine("=> Average calculation time (ms): " + time_average_ms.ToString());

        station.Delete();

        return;




        double[][] dhm;
        Mat pose_base;
        Mat pose_tool;
        // get the current robot parameters:
        /* RDK.getRobotParams(ROBOT, out dhm, out pose_base, out pose_tool);

         // change the mastering values:
         for (int i = 0; i < 6; i++)
         {
             dhm[i][2] = dhm[i][2] + 1 * Math.PI / 180.0; // change theta i (mastering value, add 1 degree)
         }

         // change the base and tool distances:
         dhm[0][3] = dhm[0][3] + 5; // add 5 mm to d1
         dhm[5][3] = dhm[5][3] + 5; // add 5 mm to d6

         // update the robot parameters back:
         RDK.setRobotParams(ROBOT, dhm, pose_base, pose_tool);*/

        return;

        // Example to rotate the view around the Z axis
        /*RoboDK.Item item_robot = RDK.ItemUserPick("Select the robot you want", RoboDK.ITEM_TYPE_ROBOT);
        item_robot.MoveL(item_robot.Pose() * Mat.transl(0, 0, 50));
        return;*/


        RDK.setViewPose(RDK.ViewPose() * Mat.rotx(10 * 3.141592 / 180));
        return;

        //---------------------------------------------------------
        // Sample to generate a program using a C# script
        if (ROBOT != null && ROBOT.Valid())
        {
            //ROBOT.Finish();
            //RDK.Finish();
            // RDK.Connect(); // redundant
            RDK.Finish(); // ignores any previous activity to generate the program
            RDK.setRunMode(RoboDK.RUNMODE_MAKE_ROBOTPROG); // Very important to set first
            RDK.ProgramStart("TestProg1", "C:\\Users\\Albert\\Desktop\\", "KAIRO.py", ROBOT);
            double[] joints1 = new double[6] { 1, 2, -50, 4, 5, 6 };
            double[] joints2 = new double[6] { -1, -2, -50, 4, 5, 6 };

            ROBOT.MoveJ(joints1);
            ROBOT.MoveJ(joints2);
            ROBOT.Finish(); // provoke program generation



            RDK.Finish(); // ignores any previous activity to generate the program
            RDK.setRunMode(RoboDK.RUNMODE_MAKE_ROBOTPROG); // Very important to set first
            RDK.ProgramStart("TestProg2_no_robot", "C:\\Users\\Albert\\Desktop\\", "Fanuc_RJ3.py");
            RDK.RunProgram("Program1");
            RDK.RunCode("Output Raw code");
            RDK.Finish(); // provoke program generation



            ROBOT.Finish(); // ignores any previous activity to generate the program
            RDK.setRunMode(RoboDK.RUNMODE_MAKE_ROBOTPROG); // Very important to set first
            RDK.ProgramStart("TestProg3", "C:\\Users\\Albert\\Desktop\\", "GSK.py", ROBOT);
            double[] joints3 = new double[6] { 10, 20, 30, 40, 50, 60 };
            double[] joints4 = new double[6] { -10, -20, -30, 40, 50, 60 };

            ROBOT.MoveJ(joints3);
            ROBOT.MoveJ(joints4);
            ROBOT.Finish(); // provoke program generation

        }
        else
        {
            Console.WriteLine("No robot selected");
        }
        return;

        //---------------------------------------------------------
        RoboDK.Item prog = RDK.getItem("", RoboDK.ITEM_TYPE_PROGRAM);
        string err_msg;
        Mat jnt_list;
        //prog.InstructionListJoints(out err_msg, out jnt_list, 0.5, 0.5);
        prog.InstructionListJoints(out err_msg, out jnt_list, 5, 5);
        for (int j = 0; j < jnt_list.cols; j++)
        {
            for (int i = 0; i < jnt_list.rows; i++)
            {
                Console.Write(jnt_list[i, j]);
                Console.Write("    ");
            }
            Console.WriteLine("");
        }



        return;


        // Example to retrieve the selected point and normal of a surface and create a target.
        // get the robot reference frame
        RoboDK.Item robot_ref = ROBOT.getLink(RoboDK.ITEM_TYPE_FRAME);
        if (!robot_ref.Valid())
        {
            Console.WriteLine("The robot doesn't have a reference frame selected. Selecting a robot reference frame (or make a reference frame active is required to add a target).");
            return;
        }

        //var obj = RDK.getItem("box", ITEM_TYPE_OBJECT);//RDK.ItemUserPick("Select an object", ITEM_TYPE_OBJECT);
        //var obj = RDK.getItem("tide", ITEM_TYPE_OBJECT);//RDK.ItemUserPick("Select an object", ITEM_TYPE_OBJECT);

        int feature_type = -1;
        int feature_id = -1;

        // Remember the information relating to the selected point (XYZ and surface normal).
        // These values are retrieved in Absolute coordinates (with respect to the station).
        double[] point_xyz = null;
        double[] point_ijk = null;

        while (true)
        {
            var obj_selected = RDK.GetSelectedItems();
            if (obj_selected.Count == 1 && obj_selected[0].Type() == RoboDK.ITEM_TYPE_OBJECT)
            {
                var obj = obj_selected[0];
                // RDK.SetSelectedItems(); // ideally we need this function to clear the selection
                var is_Selected = obj.SelectedFeature(out feature_type, out feature_id);
                if (is_Selected && feature_type == RoboDK.FEATURE_SURFACE)
                {
                    Mat point_list;
                    string description = obj.GetPoints(feature_type, feature_id, out point_list);
                    // great, we got the point from the surface. This will be size 6x1
                    Console.WriteLine("Point information: " + description);
                    if (point_list.cols < 1 || point_list.rows < 6)
                    {
                        // something is wrong! This should not happen....
                        Console.WriteLine(point_list.ToString());
                        continue;
                    }
                    double[] value = point_list.GetCol(0).ToDoubles();
                    point_xyz = new double[] { value[0], value[1], value[2] };
                    // invert the IJK values (RoboDK provides the normal coming out of the surface but we usually want the Z axis to go into the object)
                    point_ijk = new double[] { -value[3], -value[4], -value[5] };
                    Mat obj_pose_abs = obj.PoseAbs();

                    // Calculate the point in Absolute coordinates (with respect to the station)
                    point_xyz = obj_pose_abs * point_xyz;
                    point_ijk = obj_pose_abs.Rot3x3() * point_ijk;
                    break;
                }
            }
        }


        // Calculate the absolute pose of the robot reference
        Mat ref_pose_abs = robot_ref.PoseAbs();

        // Calculate the absolute pose of the robot tool 
        Mat robot_pose_abs = ref_pose_abs * robot_ref.Pose();

        // Calculate the robot pose for the selected target and use the tool Y axis as a reference
        // (we try to get the pose that has the Y axis as close as possible as the current robot position)
        Mat pose_surface_abs = Mat.xyzijk_2_pose(point_xyz, point_ijk, robot_pose_abs.VY());

        if (!pose_surface_abs.IsHomogeneous())
        {
            Console.WriteLine("Something went wrong");
            return;
        }

        // calculate the pose of the target (relative to the reference frame)
        Mat pose_surface_rel = ref_pose_abs.inv() * pose_surface_abs;

        // add a target and update the pose
        var target_new = RDK.AddTarget("T1", robot_ref, ROBOT);
        target_new.setAsCartesianTarget();
        target_new.setJoints(ROBOT.Joints()); // this is only important if we want to remember the current configuration
        target_new.setPose(pose_surface_rel);








        /*RoboDK.Item frame = RDK.getItem("FrameTest");
        double[] xyzwpr = { 1000.0, 2000.0, 3000.0, 12.0 * Math.PI / 180.0, 84.98 * Math.PI / 180.0, 90.0 * Math.PI / 180.0 };
        Mat pose;
        pose = Mat.FromUR(xyzwpr);
        double[] xyzwpr_a = pose.ToUR();
        double[] xyzwpr_b = pose.ToUR_Alternative();

        Console.WriteLine("Option one:");
        Console.Write(Mat.FromUR(xyzwpr_a).ToString());
        Console.Write(xyzwpr_a[0]); Console.WriteLine("");
        Console.Write(xyzwpr_a[1]); Console.WriteLine("");
        Console.Write(xyzwpr_a[2]); Console.WriteLine("");
        Console.Write(xyzwpr_a[3] * 180.0 / Math.PI); Console.WriteLine("");
        Console.Write(xyzwpr_a[4] * 180.0 / Math.PI); Console.WriteLine("");
        Console.Write(xyzwpr_a[5] * 180.0 / Math.PI); Console.WriteLine("");

        Console.WriteLine("Option Two:");
        Console.Write(Mat.FromUR(xyzwpr_b).ToString());
        Console.Write(xyzwpr_b[0]); Console.WriteLine("");
        Console.Write(xyzwpr_b[1]); Console.WriteLine("");
        Console.Write(xyzwpr_b[2]); Console.WriteLine("");
        Console.Write(xyzwpr_b[3] * 180.0 / Math.PI); Console.WriteLine("");
        Console.Write(xyzwpr_b[4] * 180.0 / Math.PI); Console.WriteLine("");
        Console.Write(xyzwpr_b[5] * 180.0 / Math.PI); Console.WriteLine("");*/

    }


    public bool Check_RDK()
    {
        // check if the RDK object has been initialised:
        if (RDK == null)
        {
            notifybar.Text = "RoboDK가 시작되지 않았습니다";
            return false;
        }

        // Check if the RDK API is connected
        if (!RDK.Connected())
        {
            notifybar.Text = "Connecting to RoboDK...";
            // Attempt to connect to the RDK API
            if (!RDK.Connect())
            {
                notifybar.Text = "Problems using the RoboDK API. The RoboDK API is not available...";
                return false;
            }
        }
        return true;
    }


    public bool Check_ROBOT(bool ignore_busy_status = false)
    {
        if (!Check_RDK())
        {
            return false;
        }
        if (ROBOT == null || !ROBOT.Valid())
        {
            notifybar.Text = "A robot has not been selected. Load a station or a robot file first.";
            return false;
        }
        try
        {
            notifybar.Text = "Using robot: " + ROBOT.Name();
        }
        catch (RoboDK.RDKException rdkex)
        {
            notifybar.Text = "The robot has been deleted: " + rdkex.Message;
            return false;
        }

        // Safe check: If we are doing non blocking movements, we can check if the robot is doing other movements with the Busy command
        if (!MOVE_BLOCKING && (!ignore_busy_status && ROBOT.Busy()))
        {
            notifybar.Text = "The robot is busy!! Try later...";
            return false;
        }
        return true;
    }


   
    private void Window_Loaded(object sender, RoutedEventArgs e)
    {
      
        if (!Check_RDK())
        {
            RDK = new RoboDK();
            if (!Check_RDK())
            {
                notifybar.Text = "RoboDK를 시작할 수 없습니다.";
                return;
            }
            notifybar.Text = "RoboDK is Running...";
        }
        rad_RoboDK_Integrated.IsChecked = true;
        RDK.setWindowState((int)RoboDk.API.Model.WindowState.Cinema);
        string processIdStr = RDK.Command("MainProcess_ID");
        int processId = Convert.ToInt32(processIdStr);

        EmbedProcessWindow(processId);
        this.Icon = (ImageSource)System.Windows.Application.Current.Resources["IconRoboDK"];

    }
    private void EmbedProcessWindow(int processId)
    {
        Process process = Process.GetProcessById(processId);
        if (process != null && !process.HasExited)
        {
            _hWnd = process.MainWindowHandle;
            if (_hWnd != IntPtr.Zero)
            {
                IntPtr hostHandle = new WindowInteropHelper(this).Handle;

                SetParent(_hWnd, hostHandle);

                MoveWindow(_hWnd, (int)border1.Margin.Left, (int)border1.Margin.Top, (int)border1.ActualWidth, (int)border1.ActualHeight, true);
            }
        }
    }

    //IconRoboDK


    private void RoboDK_Lock(object sender, EventArgs e)
    {
        // 잠금 처리 로직
    }

    private void RoboDK_Unlock(object sender, EventArgs e)
    {
        // 잠금 해제 처리 로직
    }



    private void btnGetJoints_Click(object sender, RoutedEventArgs e)
    {
        if (!Check_ROBOT(true)) { return; }

        double[] joints = ROBOT.Joints();
        Mat pose = ROBOT.Pose();

        // update the joints
        string strjoints = Values_2_String(joints);
        txtJoints.Text = strjoints;

        // update the pose as xyzwpr
        double[] xyzwpr = pose.ToTxyzRxyz();
        string strpose = Values_2_String(xyzwpr);
        txtPosition.Text = strpose;
    }

    public string Values_2_String(double[] dvalues)
    {
        if (dvalues == null || dvalues.Length < 1)
        {
            return "Invalid values";
        }
        // Not supported on .NET Framework 2.0:
        //string strvalues = String.Join(" , ", dvalues.Select(p => p.ToString("0.0")).ToArray());
        string strvalues = dvalues[0].ToString("0.0");
        for (int i = 1; i < dvalues.Length; i++)
        {
            strvalues += " , " + dvalues[i].ToString("0.0");
        }

        return strvalues;
        //return "";
    }

    private void btnOLPdone_Click(object sender, RoutedEventArgs e)
    {
        if (!Check_ROBOT()) return;

        // 프로그램 생성 트리거 (로봇 소켓 종료)
        ROBOT.Finish();

        // 시뮬레이션 모드로 복귀
        rad_RunMode_Simulation.IsChecked = true;
    }

    private void btnMoveJoints_Click(object sender, RoutedEventArgs e)
    {
        // 텍스트에서 로봇 조인트 값 추출 및 유효성 검사
        double[] joints = String_2_Values(txtJoints.Text);

        // RDK 실행 및 입력값 확인
        if (!Check_ROBOT() || joints == null) return;

        try
        {
            // 조인트 이동 명령
            ROBOT.MoveJ(joints, MOVE_BLOCKING);
        }
        catch (RoboDK.RDKException rdkex)
        {
            notifybar.Text = "Problems moving the robot: " + rdkex.Message;
        }
    }

    private void btnMovePose_Click(object sender, RoutedEventArgs e)
    {
        // 텍스트에서 로봇 위치 값 추출 및 유효성 검사
        double[] xyzwpr = String_2_Values(txtPosition.Text);

        // RDK 실행 및 입력값 확인
        if (!Check_ROBOT() || xyzwpr == null) return;

        // 위치값을 4x4 행렬로 변환
        Mat pose = Mat.FromTxyzRxyz(xyzwpr);
        try
        {
            ROBOT.MoveJ(pose, MOVE_BLOCKING);
        }
        catch (RoboDK.RDKException rdkex)
        {
            notifybar.Text = "Problems moving the robot: " + rdkex.Message;
        }
    }


    private void btnRun_Program_Click(object sender, EventArgs e)
    {
        // Check that there is a link with RoboDK:
        if (!Check_RDK()) { return; }

        string progname = txtRunProgram.Text;

        // Retrieve the program item
        RoboDK.Item prog = RDK.getItem(progname);

        if (prog.Valid() && (prog.Type() == RoboDK.ITEM_TYPE_PROGRAM_PYTHON || prog.Type() == RoboDK.ITEM_TYPE_PROGRAM))
        {
            // 실행 모드에 따라 RunType 지정
            if (rad_RunMode_Online.IsChecked == true)
            {
                prog.setRunType(RoboDK.PROGRAM_RUN_ON_ROBOT);
                notifybar.Text = "Running program on real robot: " + progname;
            }
            else if (rad_RunMode_Program.IsChecked == true)
            {
                prog.setRunType(RoboDK.PROGRAM_RUN_ON_SIMULATOR);
                ROBOT.RunCodeCustom(progname);
                notifybar.Text = "Generating offline program call: " + progname;
            }
            else // Simulation
            {
                prog.setRunType(RoboDK.PROGRAM_RUN_ON_SIMULATOR);
                notifybar.Text = "Running program in simulation: " + progname;
            }

            prog.RunProgram();
        }
        else
        {
            notifybar.Text = "The program " + progname + " does not exist or is not a valid program.";
        }
    }



    public double[] String_2_Values(string strvalues)
    {
        double[] dvalues = null;
        try
        {
            dvalues = Array.ConvertAll(strvalues.Split(','), Double.Parse);
        }
        catch (System.FormatException ex)
        {
            notifybar.Text = "Invalid input: " + strvalues;
        }
        return dvalues;
    }

    private void btnRun_Program_Click(object sender, RoutedEventArgs e)
    {

    }

    private void btnLoadFile_Click(object sender, EventArgs e)
    {

        if (!Check_RDK()) { return; }

        // Show the File dialog to select a file:

        var select_file = new Microsoft.Win32.OpenFileDialog();

        select_file.Title = "Select a file to open with RoboDK";
        select_file.InitialDirectory = RDK.getParam("PATH_LIBRARY").Replace("/", "\\"); // open the RoboDK library by default

        if (select_file.ShowDialog() == true)

        {
            string filename = select_file.FileName;

            // .rdk 파일이면 기존 스테이션 모두 닫기
            //if (filename.EndsWith(".rdk", StringComparison.InvariantCultureIgnoreCase))
            //{
            //    CloseAllStations();
            //}

            // 파일 추가
            RoboDK.Item item = RDK.AddFile(filename);
            if (item.Valid())
            {
                notifybar.Text = "Loaded: " + item.Name();
                // 로봇 자동 선택 시도
                SelectRobot();
            }
            else
            {
                notifybar.Text = "Could not load: " + filename;
            }
        }
    }




    private void Incremental_Move(string button_name)
    {
        if (!Check_ROBOT()) { return; }

        notifybar.Text = "Button selected: " + button_name;

        if (button_name.Length < 3)
        {
            notifybar.Text = "Internal problem! Button name should be like +J1, -Tx, +Rz or similar";
            return;
        }

        // 변경
        if (string.IsNullOrWhiteSpace(numStep.Text) || !double.TryParse(numStep.Text, out double stepValue))
        {
            notifybar.Text = "Step 값이 비어 있거나 잘못되었습니다.";
            return;
        }
        double move_step = 0.0;
        if (button_name[0] == '+')
        {
            move_step = +stepValue;
        }
        else if (button_name[0] == '-')
        {
            move_step = -stepValue;
        }


        //////////////////////////////////////////////
        //////// if we are moving in the joint space:
        if (rad_Move_Joints.IsChecked == true)
        {
            double[] joints = ROBOT.Joints();

            // get the moving axis (1, 2, 3, 4, 5 or 6)
            int joint_id = Convert.ToInt32(button_name[2].ToString()) - 1; // important, double array starts at 0

            if (joint_id < 0 || joint_id >= joints.Length)
            {
                notifybar.Text = "잘못된 조인트 번호입니다.";
                return;
            }

            joints[joint_id] = joints[joint_id] + move_step;

            try
            {
                ROBOT.MoveJ(joints, MOVE_BLOCKING);
            }
            catch (RoboDK.RDKException rdkex)
            {
                notifybar.Text = "The robot can't move to the target joints: " + rdkex.Message;
            }
        }
        else
        {
            //////////////////////////////////////////////
            //////// if we are moving in the cartesian space
            // Button name examples: +Tx, -Tz, +Rx, +Ry, +Rz

            int move_id = -1;
            string[] move_types = new string[6] { "Tx", "Ty", "Tz", "Rx", "Ry", "Rz" };

            for (int i = 0; i < 6; i++)
            {
                if (button_name.EndsWith(move_types[i]))
                {
                    move_id = i;
                    break;
                }
            }
            if (move_id == -1)
            {
                notifybar.Text = "Internal problem! Unknown move type.";
                return;
            }

            double[] move_xyzwpr = new double[6] { 0, 0, 0, 0, 0, 0 };
            move_xyzwpr[move_id] = move_step;
            Mat movement_pose = Mat.FromTxyzRxyz(move_xyzwpr);

            // the current position of the robot (as a 4x4 matrix)
            Mat robot_pose = ROBOT.Pose();

            // Calculate the new position of the robot
            Mat new_robot_pose;
            bool is_TCP_relative_move = rad_Move_wrt_Tool.IsChecked == true;
            if (is_TCP_relative_move)
            {
                // if the movement is relative to the TCP we must POST MULTIPLY the movement
                new_robot_pose = robot_pose * movement_pose;
            }
            else
            {
                // if the movement is relative to the reference frame we must PRE MULTIPLY the XYZ translation:
                // Note: Rotation applies from the robot axes.

                Mat transformation_axes = new Mat(robot_pose);
                transformation_axes.setPos(0, 0, 0);
                Mat movement_pose_aligned = transformation_axes.inv() * movement_pose * transformation_axes;
                new_robot_pose = robot_pose * movement_pose_aligned;
            }

            // Then, we can do the movement:
            try
            {
                ROBOT.MoveJ(new_robot_pose, MOVE_BLOCKING);
            }
            catch (RoboDK.RDKException rdkex)
            {
                notifybar.Text = "The robot can't move to " + new_robot_pose.ToString();
            }
        }
    }

    private void btnTXpos_Click(object sender, RoutedEventArgs e)
    {

    }

    private void btnTXneg_Click(object sender, RoutedEventArgs e)
    {

    }

    private void btnStepUp_Click(object sender, RoutedEventArgs e)
    {
        if (int.TryParse(numStep.Text, out int value))
        {
            value = Math.Min(StepMax, value + StepIncrement);
            numStep.Text = value.ToString();
        }
    }

    private void btnStepDown_Click(object sender, RoutedEventArgs e)
    {
        if (int.TryParse(numStep.Text, out int value))
        {
            value = Math.Max(StepMin, value - StepIncrement);
            numStep.Text = value.ToString();
        }
    }

    private void numStep_PreviewTextInput(object sender, TextCompositionEventArgs e)
    {
        e.Handled = !int.TryParse(e.Text, out _);
    }

    private void numStep_LostFocus(object sender, RoutedEventArgs e)
    {
        if (int.TryParse(numStep.Text, out int value))
        {
            value = Math.Max(StepMin, Math.Min(StepMax, value));
            numStep.Text = value.ToString();
        }
        else
        {
            numStep.Text = StepMin.ToString();
        }
    }
}




