using System.ComponentModel;
using System.Diagnostics;
using System.Runtime.InteropServices;
using System.Windows;
using System.Windows.Input;
using System.Windows.Interop;
using System.Windows.Media;

using RoboDk.API;



namespace TheSystemRoboDkTest;
public partial class MainWindow : Window
{
    #region A
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


    #endregion
    public MainWindow()
    {
        InitializeComponent();
    }


    private void Window_Closing(object sender, CancelEventArgs e)
    {
            SetParent(_hWnd, IntPtr.Zero);
        RDK.Disconnect();

        RDK.CloseRoboDK();
            RDK = null;
            _hWnd = IntPtr.Zero;
        
    }


    private void Window_Loaded(object sender, RoutedEventArgs e)
    {

        if (!Check_RDK())
        {
            
            RDK = new RoboDK();
            RDK.setWindowState(RoboDK.WINDOWSTATE_HIDDEN);
            if (!Check_RDK())
            {
                notifybar.Text = "RoboDK를 시작할 수 없습니다.";
                return;
            }
            notifybar.Text = "RoboDK is Running...";
        }
        //    rad_RoboDK_Integrated.IsChecked = true;
        RDK.setWindowState(RoboDK.WINDOWSTATE_NORMAL);
        RDK.setWindowState((int)RoboDk.API.Model.WindowState.Cinema);
        string processIdStr = RDK.Command("MainProcess_ID");
        int processId = Convert.ToInt32(processIdStr);

        EmbedProcessWindow(processId);

        this.Icon = (ImageSource)System.Windows.Application.Current.Resources["IconRoboDK"];

    }

    public void CloseAllStations()
    {
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
        if (sender is System.Windows.Controls.RadioButton rb && rb.IsChecked != true)
            return;

        if (!Check_RDK())
            return;

        SetParent(RDK.GetWindowHandle(), IntPtr.Zero);

        //RDK.setWindowState(RoboDK.WINDOWSTATE_NORMAL);      // 일반 모드
        //RDK.setWindowState(RoboDK.WINDOWSTATE_MAXIMIZED);   // 최대화

        //this.Height = this.MinHeight;
        //this.Width = this.MinWidth;

        this.Activate();

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
 
        this.Width = this.MinWidth;
        this.Height = this.MinHeight;
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


   

    private void EmbedProcessWindow(int processId)
    {

        var relativePoint = border1.TransformToAncestor(this).Transform(new System.Windows.Point(0, 0));
        int x = (int)relativePoint.X;
        int y = (int)relativePoint.Y;
        int width = (int)border1.ActualWidth;
        int height = (int)border1.ActualHeight;



        Process process = Process.GetProcessById(processId);
        if (process != null && !process.HasExited)
        {
            _hWnd = process.MainWindowHandle;
            if (_hWnd != IntPtr.Zero)
            {
                IntPtr hostHandle = new WindowInteropHelper(this).Handle;

                SetParent(_hWnd, hostHandle);
                MoveWindow(_hWnd, x+30, y+5, width+121, height+120, true);
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
       
        Incremental_Move("+Tx");
    }

    private void btnTXneg_Click(object sender, RoutedEventArgs e)
    {
        
            Incremental_Move("-Tx");
        
    
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




