



using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Windows.Forms;
using System.Runtime.InteropServices;
using System.IO.Ports;
using System.IO;
using ArdupilotMega;
using System.Xml;
using System.Globalization;
using System.Text.RegularExpressions;
using System.Net;
using System.Diagnostics;
using System.Threading;



namespace OSD {

    using uint32_t = System.UInt32;
    using uint16_t = System.UInt16;
    using uint8_t = System.Byte;
    using int8_t = System.SByte;
    //using boolean = System.Byte;

    public partial class Stand : Form {

        public const int PORT_SPEED = 57600; 
        //*****************************************/		
        public const string VERSION = "v0.1";

        //max 7456 datasheet pg 10
        //pal  = 16r 30 char
        //ntsc = 13r 30 char
        public const int CHAR_W = 12;
        public const int CHAR_H = 18;

        public const int SCREEN_W = 30;
        public const int SCREEN_H = 16;
        public const int SCREEN_H_NTSC = 13;

        public const int MAVLINK_MAX_PAYLOAD_LEN =255;
        public const int MAVLINK_NUM_CHECKSUM_BYTES =2;
        public const int MAVLINK_STX =254;
        public const int X25_INIT_CRC = 0xffff;

        public const int NUM_PWM_Channels = 8;

        public enum ModelType {
            Plane = 0,
            Copter = 1,
            Unknown = 9
        };

        public struct mavlink_message {
            public uint16_t checksum; /// sent at end of packet
            public uint8_t magic;   /// protocol magic marker
            public uint8_t len;     /// Length of payload
            public uint8_t seq;     /// Sequence of packet
            public uint8_t sysid;   /// ID of message sender system/aircraft
            public uint8_t compid;  /// ID of the message sender component
            public uint8_t msgid;   /// ID of message in payload
            public uint8_t[] payload; // MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES
        };

        public enum mavlink_parse_state_t {
            MAVLINK_PARSE_STATE_UNINIT=0,
            MAVLINK_PARSE_STATE_IDLE,
            MAVLINK_PARSE_STATE_GOT_STX,
            MAVLINK_PARSE_STATE_GOT_SEQ,
            MAVLINK_PARSE_STATE_GOT_LENGTH,
            MAVLINK_PARSE_STATE_GOT_SYSID,
            MAVLINK_PARSE_STATE_GOT_COMPID,
            MAVLINK_PARSE_STATE_GOT_MSGID,
            MAVLINK_PARSE_STATE_GOT_PAYLOAD,
            MAVLINK_PARSE_STATE_GOT_CRC1,
            MAVLINK_PARSE_STATE_BAD_CRC1
        }; ///< The state machine for the comm parser

        public struct mavlink_status {
            public uint8_t msg_received;               /// Number of received messages
            public uint8_t msg_error;                  /// Number of error messages
            public uint8_t buffer_overrun;             /// Number of buffer overruns
            public uint8_t parse_error;                /// Number of parse errors
            public mavlink_parse_state_t parse_state;  /// Parsing state machine
            public uint8_t packet_idx;                 /// Index in current packet
            public uint8_t current_rx_seq;             /// Sequence number of last packet received
            public uint8_t current_tx_seq;             /// Sequence number of last packet sent
            public uint16_t packet_rx_success_count;   /// Received packets
            public uint16_t packet_rx_drop_count;      /// Number of packet drops
        };

        public mavlink_status status = new mavlink_status() { msg_received = 0, msg_error=0, buffer_overrun = 0, parse_error = 0, parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE, packet_idx = 0, current_rx_seq = 0, current_tx_seq = 0, packet_rx_success_count = 0, packet_rx_drop_count = 0 };

        public mavlink_message rxmsg;

        bool started=false;

        public const int npanel = 4; // количество панелей 

        /*------------------------------------------------*/
        public int screen_number = 0;

        const Int16 toggle_offset = 2;
        public Size basesize = new Size(Stand.SCREEN_W, SCREEN_H);
        /// <summary>
        /// the un-scaled font render image
        /// </summary>
        public Bitmap screen = new Bitmap(SCREEN_W * CHAR_W, SCREEN_H * CHAR_H);
        /// <summary>
        /// the scaled to size background control
        /// </summary>
        Bitmap image = new Bitmap(SCREEN_W * CHAR_W, SCREEN_H * CHAR_H);
        Graphics gr;

        /// <summary>
        /// Bitmaps of all the chars created from the mcm
        /// </summary>
        Bitmap[] chars;
        /// <summary>
        /// record of what panel is using what squares
        /// </summary>
        public string[][] usedPostion = new string[SCREEN_W][];
        /// <summary>
        /// used to track currently selected panel across calls
        /// </summary>
        public string currentlyselected;
        /// <summary>
        /// used to track current processing panel across calls (because i maintained the original code for panel drawing)
        /// </summary>
        string processingpanel = "";
        /// <summary>
        /// use to draw the red outline box is currentlyselected matchs
        /// </summary>
        bool selectedrectangle = false;
        /// <summary>
        /// use to as a invalidator
        /// </summary>
        bool startup = false;

        /// <summary>
        /// background image
        /// </summary>
        Image bgpicture;
       


        public SerialPort comPort = new SerialPort();

      
        int osd_functions_N = 0;

        public int panel_num = 0;

        internal Config conf;

        int print_x; // текущие координаты вывода
        int print_y;

        const double PI=3.1415926535;

        Boolean fwWasRead = false;
        private bool tlog_run = false;
        public byte[] tlog_data;
        private bool need_stop_tlog=false;

        System.Threading.Thread tlog_thread;
        public System.Threading.Thread com_thread;
        public bool com_run=false;

        bool RSSI_used = false; //  использование дополнительных ног
        bool curr_used = false;
        bool batt1_used = false;
        bool batt2_used = false;

        bool fListen=false;

        bool flag_EEPROM_read=false;
        private bool[] mav_blocks = new bool[4096 / 128];

        string CurrentCOM;

        public string[] StringArray = new string[128];

        public Stand osdx;

        double startTime;

        private int MAV_packetcount=0;
        volatile object objlock = new object();

        bool fDone=false;
        bool comBusy=false;

        public Stand() {

            conf = new Config(this); // конфиг по умолчанию
            osdx=this;


            rxmsg.payload = new Byte[MAVLINK_MAX_PAYLOAD_LEN + MAVLINK_NUM_CHECKSUM_BYTES];
            
            status.packet_rx_drop_count = 0;
            status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
            status.packet_idx = 0;
            status.packet_rx_success_count = 0;
            status.current_rx_seq = 0;
            status.buffer_overrun = 0;
            status.parse_error = 0;


            InitializeComponent();

        }


        public string[] GetPortNames() {
            string[] devs = new string[0];

            if (Directory.Exists("/dev/"))
                devs = Directory.GetFiles("/dev/", "*ACM*");

            string[] ports = SerialPort.GetPortNames();

            string[] all = new string[devs.Length + ports.Length];

            devs.CopyTo(all, 0);
            ports.CopyTo(all, devs.Length);

            return all;
        }


  //      public System.Threading.Thread com_thread;
        public bool can_run=false;
        int pwm=0;

        StreamWriter sw;

        private void btnStart_Click(object sender, EventArgs e) {
            started = !started;
            

            if (started) {
                if (comPort.IsOpen)
                    comPort.Close();

                try {
                    CurrentCOM = CMB_ComPort.Text;
                    comPort.PortName = CurrentCOM;
                    comPort.BaudRate = PORT_SPEED;
                    //comPort.BaudRate = 115200;


                    comPort.Open();

                    comPort.DtrEnable = true;

                } catch (Exception ex) {
                    MessageBox.Show("Error opening com port:" + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                    started = false;
                    return;
                }

                btnStart.Text = "Stop";
                //comWrite(".");                
                emptyPort();
                comWrite("f");
                emptyPort();

                can_run=true;
                com_thread = new System.Threading.Thread(thread_proc);

                pwm=int.Parse(txtMin.Text);
                val_count =0;
                num_cols=0;

                string p=Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar;
                int i=0;
                string fn;

                do {
                    fn = p +  txtName.Text + (i!=0?i.ToString():"") + ".txt";
                    i++;
                }  while(File.Exists(fn));

                sw = new StreamWriter(fn);

                com_thread.Start();

            } else {
                doClose();
            }
        }

        void emptyPort(){
            double t=millis();
            while(millis()-t < 100){
                if(myReadLine()!="")
                    t=millis();
            }
        }

        void comWrite(string s){
            if(comPort.IsOpen ){
                Console.WriteLine("<" + s);
                addConsole("<" + s);
               comPort.WriteLine(s);
            }
        }

        void doClose(){
            sw.Close();
            btnStart.Text = "Start";
            started=false;
            
            can_run=false;
            System.Threading.Thread.Sleep(50);

            comWrite(".");
            comPort.Close();
        }

        void thread_proc() {

            double t=millis();
            int step = int.Parse(txtStep.Text);
            int max = int.Parse(txtMax.Text);
            int steptime = int.Parse(txtStepTime.Text); 
            
            while (can_run) {
                comWrite(pwm.ToString());

                while ((millis() - t < steptime || val_count == 0) && can_run) {
                    parsePort();
                    if(millis() - t > steptime*5) break; // no data for long time
                }

                if (pwm >= max) {
                    this.BeginInvoke((MethodInvoker)delegate {
                        doClose();
                    });

                    break;
                }

                if(val_count>0)
                    pushData();

                t=millis();
                pwm+=step;
                this.BeginInvoke((MethodInvoker)delegate {
                    txtPWM.Text = pwm.ToString();
                });
                
            }

        }

        string buf="";
        double[] vals = new double[10];
        int val_count=0;
        int num_cols=0;

        private string myReadLine() {
           while (comPort.BytesToRead != 0) {
               char c = (char)comPort.ReadByte();
               if (c == '\n' || (c == '\r' && buf.Length != 0)) {
                   string s=buf;
                   buf="";
                   Console.WriteLine (">"+s);
                   addConsole(">"+s);
                   if(s.Length>0 && s[0]=='#') return "";
                   return s;
               }
               if(c == '\r') continue;
               buf += c;            
            }
            return "";
        }
       

        void parsePort(){
            string s = myReadLine();

            if(s!="") {
                string [] ss=s.Split(new char[] { ' ' });
                try {
                    for(int i=0; i<ss.Length ;i++){                    
                        float f=myConvert (ss[i]);
                        vals[i]+=f;
                        if(i>num_cols) num_cols=i;

                        this.BeginInvoke((MethodInvoker)delegate {
                            Control c = this.Controls.Find("txtData" + i.ToString(), true).FirstOrDefault() as Control;
                            c.Text = f.ToString();
                        });

                    }
                    val_count++;
                } catch (Exception ex) {
                    Console.WriteLine (ex.Message );
                }
            }
        }

        void pushData(){
             if(val_count>0){
                for(int i=0;i<num_cols+1;i++){
                    if(i>0) sw.Write ("\t");

                    vals[i]/=val_count;
                    sw.Write (vals[i]);

                    vals[i]=0;
                }
                sw.WriteLine();

                val_count=0;
             }
        }


        private void btnCalib_Click(object sender, EventArgs e) {
            try {
                CurrentCOM = CMB_ComPort.Text;
                comPort.PortName = CurrentCOM;
                comPort.BaudRate = PORT_SPEED;
                //comPort.BaudRate = 115200;

                if(!comPort.IsOpen )
                    comPort.Open();

                comPort.DtrEnable = true;

            } catch(Exception ex) {
                MessageBox.Show("Error opening com port:"+ex.Message , "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                started = false;
                return;
            }


            btnCalib.Enabled =false ;
            //comWrite(".");
            emptyPort();
            
            comWrite("r");
            double t=millis();
            while(millis()-t < 5000){
                if(myReadLine ()!="") break;
            }
            btnCalib.Enabled = true;

            emptyPort();
            comPort.Close ();
        }


        private void btnSend_Click(object sender, EventArgs e) {
            try {
                CurrentCOM = CMB_ComPort.Text;
                comPort.PortName = CurrentCOM;
                comPort.BaudRate = PORT_SPEED;
                //comPort.BaudRate = 115200;

                if (!comPort.IsOpen)
                    comPort.Open();

                comPort.DtrEnable = true;

            } catch (Exception ex) {
                MessageBox.Show("Error opening com port:" + ex.Message, "Error", MessageBoxButtons.OK, MessageBoxIcon.Error);
                started = false;
                return;
            }

            
            comWrite(txtCmd.Text);

            emptyPort();

            comPort.Close();
        }

       
        void addConsole(string s){
            var i = lstConsole.Items.Add(s);
            if(lstConsole.Items.Count >1000)
                lstConsole.Items.Remove(lstConsole.Items[0]);
            //lstConsole.SelectedItem = lstConsole.Items[i];
            lstConsole.SelectedIndex = i;
        }







        /*



                struct Vect {
                    public double x;
                    public double y;
                    public double z;
                };

                const int Data_Size = 300;// width of scales

                Vect[] data_a = new Vect[Data_Size];
                Vect[] data_m = new Vect[Data_Size];
                Vect[] data_g = new Vect[Data_Size];

                double igx=0, igy=0, igz=0;
                ulong last=0;

                double tiltAngle = 90;       // Tilt angle
       

                double rollAngle = 0;        // Roll angle
        

                double panAngle = 90;        // Pan angle
        

                double offs_x=0,offs_y=0,offs_z=0;// gyro offsets
                int off_cnt=0;

                Vect from_euler(double roll, double pitch, double yaw) {
                    double cp = Math.Cos(pitch);
                    double sp = Math.Sin(pitch);
                    double sr = Math.Sin(roll);
                    double cr = Math.Cos(roll);
                    double sy = Math.Sin(yaw);
                    double cy = Math.Cos(yaw);

                    Vect a,b,c;
                    a.x = cp * cy;
                    a.y = (sr * sp * cy) - (cr * sy);
                    a.z = (cr * sp * cy) + (sr * sy);
                    b.x = cp * sy;
                    b.y = (sr * sp * sy) + (cr * cy);
                    b.z = (cr * sp * sy) - (sr * cy);
                    c.x = -sp;
                    c.y = sr * cp;
                    c.z = cr * cp;

                    Vect v = new Vect{ x=1, y=0, z=0}; // 1-vector

                    Vect ret = new Vect{ x=a.x * v.x + a.y * v.y + a.z * v.z,
                                         y=b.x * v.x + b.y * v.y + b.z * v.z,
                                         z=c.x * v.x + c.y * v.y + c.z * v.z};
                    return ret;
                }


                void DrawCompass(double ax, double ay, double az, double mx, double my, double mz, ulong time, double gx, double gy,double gz) {
            
                    tAx.Text = ax.ToString();
                    tAy.Text = ay.ToString();
                    tAz.Text = az.ToString();

                    tMx.Text = mx.ToString();
                    tMy.Text = my.ToString();
                    tMz.Text = mz.ToString();

                //    if(millis() - startTime < 10000) {
                        off_cnt++;
                        offs_x += gx;
                        offs_y += gy;
                        offs_z += gz;
                    //}

            
                    ulong dt = time-last;
            
                    double  ngx=(gx - offs_x/off_cnt) * dt / 1000000,
                            ngy=(gy - offs_y/off_cnt) * dt / 1000000,
                            ngz=(gz - offs_z/off_cnt) * dt / 1000000;

                    igx += ngx;
                    igy += ngy;
                    igz += ngz;

                    last = time;

                    const double FROM_GRADUS = 3.1415926535 / 180, TO_GRADUS = 1 / FROM_GRADUS;

                    tGx.Text = igx.ToString();
                    tGy.Text = igy.ToString();
                    tGz.Text = igz.ToString();

                    double gl = Math.Sqrt(igx * igx + igy * igy + igz * igz);

            
                    //    1.get pane perpendicular to gravity 
                    //  2.project magnetic vector to it
             
                    double al = Math.Sqrt(ax*ax + ay*ay + az*az);
            
                    ax /= al;
                    ay /= al;
                    az /= al;

                    double ml = Math.Sqrt(mx*mx + my*my + mz*mz);
                    mx /= ml;
                    my /= ml;
                    mz /= ml;

                    double tilt = (tiltAngle - 90) * FROM_GRADUS,
                           roll = (rollAngle - 90) * FROM_GRADUS;

                    double cos_tilt = Math.Cos(tilt);
                    double sin_tilt = Math.Sin(tilt);
                    double cos_roll = Math.Cos(roll);
                    double sin_roll = Math.Sin(roll);

            
                    // corrected
                    double cmx  = mx * cos_tilt
                                + mz * sin_tilt;

                    double cmy =  mx * sin_roll * sin_tilt
                                + my * cos_roll
                                - mz * sin_roll * cos_tilt;

                    double cmz = -mx * cos_roll * sin_tilt
                                + my * sin_roll
                                + mz * cos_roll * cos_tilt;
             
            
                    const double gyroWeightTiltRoll = 1-0.0001,gyroWeightPan=1-0.0001;

                    double p, v, accAngle_0, accAngle_1;


                    p = -ax ;
                    v = 0;
                    if (p < -1)
                        accAngle_0 = -90;
                    else if (p > 1)
                        accAngle_0 = 90;
                    else {
                        v = Math.Asin(p);
                        accAngle_0 = v * TO_GRADUS;
                    }

                    p = ay  / Math.Cos(v);

                    if (accAngle_0 == -90 || accAngle_0 == 90)
                        accAngle_1 = 0;
                    else if (p < -1)
                        accAngle_1 = -90;
                    else if (p > 1)
                        accAngle_1 = 90;
                    else
                        accAngle_1 = Math.Asin(p) * TO_GRADUS;

                    double magA;
                    if (mx == 0 && my == 0)
                        magA = 90;
                    else {
                        // Calculate pan-angle from magnetometer. 
                        //      magAngle[2] = atan2(mx, my) * TO_GRADUS + 90;
                        magA = Math.Atan2(my, mx) * TO_GRADUS + 90;
                    }

                    if (magA > 180) {
                        magA -= 360;
                    } else if (magA < -180) {
                        magA += 360;
                    }

                    // Simple FilterSensorData, uses mainly gyro-data, but uses accelerometer to compensate for drift
                    rollAngle = (rollAngle + (ngx * cos_tilt + ngz * sin_tilt)) * gyroWeightTiltRoll + accAngle_1 * (1 - gyroWeightTiltRoll);
                    tiltAngle = (tiltAngle + (ngy *  cos_roll +  ngz * -sin_roll)                      ) * gyroWeightTiltRoll + accAngle_0 * (1 - gyroWeightTiltRoll);
                    panAngle =  (panAngle +  (ngz *  cos_tilt + (ngx * -sin_tilt) + (ngy * sin_roll))  ) * gyroWeightPan      + magA        * (1 - gyroWeightPan);


                    drawArrow(picAcc, ax,ay,az, 1);
                    drawArrow(picMag, -mx, -my, -mz, 2);
            
                    drawGraph(picGraphA, data_a, ax, ay, az);
                    drawGraph(picGraphM, data_m, mx, my, mz);

                    if(chkRaw.Checked) {                
                        Vect vv = from_euler(igx / gl, igy / gl, igz / gl);
                        drawArrow(picGyr, vv.x, vv.y, vv.z, 0);
                        drawGraph(picGraphG, data_g, igx/gl, igy/gl, igz/gl);
                    } else {
                    }

                }

                void drawAtt(double roll, double pitch, double yaw) {
                    if(!chkRaw.Checked) {
                        Vect vv = from_euler(roll, pitch, yaw);
                        drawArrow(picGyr, -vv.x, -vv.y, -vv.z, 2);
                        drawGraph(picGraphG, data_g, roll / PI, pitch / PI, yaw / PI);
                    }
                }


                void drawArrow(System.Windows.Forms.PictureBox pb, double x, double y, double z, int mode){
                    int x0 = pb.Width / 2;
                    int y0 = pb.Height / 2;
          
                    image = new Bitmap(pb.Width, pb.Height);

                    Graphics grfull = Graphics.FromImage(image);
                    grfull.DrawArc(new Pen(Color.Black, 1),0,0,pb.Width-1 , pb.Height-1 ,0,360);

                    grfull.DrawLine(new Pen(Color.Gray, 1),  x0, 0, x0, pb.Height);
                    grfull.DrawLine(new Pen(Color.Gray, 1), 0, y0, pb.Width, y0);

                    switch(mode){
                    case 0: // none
                        break ;
                    case 1: // arrow
                        grfull.DrawLine(new Pen(Color.Gray, 1), 0, y0, 10, y0 + 10);
                        grfull.DrawLine(new Pen(Color.Gray, 1), 0, y0, 10, y0 - 10);
                        break;
                    case 2: //
                        grfull.DrawLine(new Pen(Color.Gray, 1), 4,  y0 - 10, 4,  y0 + 10);
                        grfull.DrawLine(new Pen(Color.Gray, 1), 14, y0 - 10, 14, y0 + 10);
                        grfull.DrawLine(new Pen(Color.Gray, 1), 14, y0 + 10, 4,  y0 - 10);

                        break;
                    }

                    int dx, dy;
                    try {
                        dx=(int)(x0*x);
                        dy=(int)(y0*y);
                    }catch{
                        dx=0;
                        dy=0;
                    }
                    try {
                        if(z>0)
                            grfull.DrawLine(new Pen(Color.Red, 1), x0, y0, x0 + dx, y0 + dy);
                        else
                            grfull.DrawLine(new Pen(Color.Blue, 1), x0, y0, x0 + dx, y0 + dy);
                    } catch {}

                    pb.Image = image;
                }


                void drawGraph(System.Windows.Forms.PictureBox pb, Vect[] data, double x, double y, double z) {
                    int x0 = pb.Width / 2;
                    int y0 = pb.Height / 2;

                    image = new Bitmap(pb.Width, pb.Height);
                    //pb.Tag 

                    Graphics grfull = Graphics.FromImage(image);
            
                    grfull.DrawLine(new Pen(Color.Gray, 1), 0, y0,  pb.Width, y0);
                    grfull.DrawLine(new Pen(Color.Gray, 1), 0, 0, 0, pb.Height );
                    grfull.DrawLine(new Pen(Color.Gray, 1), pb.Width - 1, 0, pb.Width-1, pb.Height);            
            
                    for(int i=0; i<data.Length ; i++ ){
                
                        double  vx=data[i].x,
                                vy=data[i].y,
                                vz=data[i].z;

                        int dx, dy, dz;
                        try {
                            dx = (int)(y0 * vx);
                            dy = (int)(y0 * vy);
                            dz = (int)(y0 * vz);
                        } catch {
                            dx = 0;
                            dy = 0;
                            dz = 0;
                        }
                
                        try {
                            //grfull.
                            grfull.DrawLine(new Pen(Color.Red, 1), i, y0-dx, i+1, y0 - dx);
                        } catch { }

                        try {
                            grfull.DrawLine(new Pen(Color.Green, 1), i, y0-dy, i+1, y0 - dy);
                        } catch { }

                        try {
                            grfull.DrawLine(new Pen(Color.Blue, 1), i, y0-dz, i+1, y0 - dz);
                        } catch { }

                        if((i+1)<data.Length)
                            data[i] = data[i+1];
                    }
            
                    data[data.Length - 1].x = x;
                    data[data.Length - 1].y = y;
                    data[data.Length - 1].z = z;
            

                    pb.Image = image;
                }
        */

        string currentVersion = "";

        private void OSD_Load(object sender, EventArgs e) {
           

            Translate(this);


            //string strVersion = System.Reflection.Assembly.GetExecutingAssembly().GetName().Version.ToString();
            this.Text = this.Text + " " + VERSION;
            currentVersion =  VERSION;

            CMB_ComPort.Items.AddRange(GetPortNames());

            if (CMB_ComPort.Items.Count > 0)
                CMB_ComPort.SelectedIndex = 0;

            xmlconfig(false);
            /*
            drawArrow(picAcc, 0, 0, 0,0);
            drawArrow(picMag, 0, 0, 0,0);
            drawArrow(picGyr, 0, 0, 0,0);
             */
        }

        bool loop=false;



        private string cnv_limit(string s){
            string fmt = convertChars(s);
            try {
                fmt = fmt.Substring(0, Config.OSD_SENSOR_FORMAT_TOTAL);
            } catch {
            }
            return fmt;
        }


        private System.Threading.Thread clear_thread=null ;
        
        private  void clear_thread_proc(){
            System.Threading.Thread.Sleep(10000); 
            
            
            clear_thread=null;
        }


/*       
        private void btnConnect_Click(object sender, EventArgs e) {
            if(comPort.IsOpen ){
                comPort.Close();
                return;
            }
            comPort.PortName = CMB_ComPort.Text;
            comPort.BaudRate = 57600;

            comPort.Open();

            comPort.DtrEnable = true;
            comPort.RtsEnable = true;

            Application.DoEvents();
 
            startTime = millis();
                
            byte[] buffer=new byte[512];
            int index=-1;

            if(comPort.IsOpen) {
                loop=true;
                int np=0;
                string message;

                while (loop && !fDone) {
                    string so = "";
                        
                    while(comPort.BytesToRead != 0 && loop) {

                        byte c = (byte)comPort.ReadByte();
                        byte[] ba = new byte[2];
                        

                        if (c == 0 || c == 10) {
                            so = "";
                        } else {
                            char ch = (char)c;
                            so = so + ch; 
                        }

                        int stx_count=0, error_count=0;
                        if(index>=0) buffer[index++]=c;

                        switch (mavlink_parse_char(c)) {
                        case 3: // error
                            error_count++;
                            Console.WriteLine("ERROR parsing "+MAVLink.MAVLINK_NAMES[buffer[5]] + "\n");
                            break;
                        case 1: // got STX
                            index=0;
                            buffer[index++] = c; // store STX
                            stx_count++;
                            break;
                        case 2: // got packet

                            Console.WriteLine(MAVLink.MAVLINK_NAMES[buffer[5]] + "\n");

                            switch(rxmsg.msgid) {
                            case (byte)MAVLink.MAVLINK_MSG_ID.RAW_IMU:
                                MAVLink.mavlink_raw_imu_t msg = ByteArrayToStructureGC<MAVLink.mavlink_raw_imu_t>(rxmsg.payload, 0);

                                DrawCompass(msg.xacc, msg.yacc, msg.zacc, msg.xmag, msg.ymag, msg.zmag, msg.time_usec, msg.xgyro, msg.ygyro, msg.zgyro );
                                break;

                            case (byte)MAVLink.MAVLINK_MSG_ID.ATTITUDE:
                                MAVLink.mavlink_attitude_t att = ByteArrayToStructureGC<MAVLink.mavlink_attitude_t>(rxmsg.payload, 0);
                                drawAtt(att.roll,att.pitch ,att.yaw );
                                break;
                            default:
                                    break;
                            }

                            index= -1;
                                
                            np++;
                                
                            Application.DoEvents();
                            break;
                        } // switch
                    } // while bytes_to_read
                    Application.DoEvents();

                }// loop

            } else { // if open
               
            }
 
            comPort.Close();
          

        }

        */
        private void OSD_FormClosed(object sender, FormClosedEventArgs e) {
            fDone=true;
            can_run =false;

            xmlconfig(true);
        }

        private bool autoUpdate = false;
        private bool checkForUpdates = true;

        private void xmlconfig(bool write) {
            if (write || !File.Exists(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml")) {
                try {
                    XmlTextWriter xmlwriter = new XmlTextWriter(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml", Encoding.ASCII);
                    xmlwriter.Formatting = Formatting.Indented;

                    xmlwriter.WriteStartDocument();

                    xmlwriter.WriteStartElement("Config");

                    xmlwriter.WriteElementString("comport", CMB_ComPort.Text);

                    xmlwriter.WriteElementString("min", txtMin.Text);
                    xmlwriter.WriteElementString("max", txtMax.Text);
                    xmlwriter.WriteElementString("step", txtStep.Text);
                    xmlwriter.WriteElementString("stepTime", txtStepTime.Text);
                    xmlwriter.WriteElementString("name", txtName.Text);
                                       

                    xmlwriter.WriteEndElement();

                    xmlwriter.WriteEndDocument();
                    xmlwriter.Close();

                    //appconfig.Save();
                } catch (Exception ex) { MessageBox.Show(ex.ToString()); }
            } else {
                try {
                    using (XmlTextReader xmlreader = new XmlTextReader(Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + @"config.xml")) {
                        while (xmlreader.Read()) {
                            xmlreader.MoveToElement();
                            try {
                                switch (xmlreader.Name) {
                                case "comport":
                                    string temp = xmlreader.ReadString();
                                    CMB_ComPort.Text = temp;
                                    break;
                                case "min":
                                    txtMin.Text = xmlreader.ReadString();
                                    break;

                                case "max":
                                    txtMax.Text = xmlreader.ReadString();
                                    break;

                                case "step":
                                    txtStep.Text = xmlreader.ReadString();
                                    break;

                                case "stepTime":
                                    txtStepTime.Text = xmlreader.ReadString();
                                    break;

                                case "name":
                                    txtName.Text = xmlreader.ReadString();
                                    break;

                                case "xml":
                                    break;
                                default:
                                    if (xmlreader.Name == "") // line feeds
                                        break;
                                    break;
                                }
                            } catch (Exception ee) { Console.WriteLine(ee.Message); } // silent fail on bad entry
                        }
                    }
                } catch (Exception ex) { Console.WriteLine("Bad Config File: " + ex.ToString()); } // bad config file
            }
        }


    

        int Constrain(int value, int min, int max) {
            if (value < min)
                return (int)min;
            if (value > max)
                return (int)max;

            return (int)value;
        }

        public static ushort crc_accumulate(byte b, ref ushort crc)    {
            unchecked        {
                byte ch = (byte)(b ^ (byte)(crc & 0x00ff));
                ch = (byte)(ch ^ (ch << 4));
                crc = (ushort)((crc >> 8) ^ (ch << 8) ^ (ch << 3) ^ (ch >> 4));
                return crc;
            }
        }
        static  void crc_init(ref uint16_t crcAccum)
        {
                crcAccum = X25_INIT_CRC;
        }

        private void mavlink_start_checksum(ref mavlink_message msg){
            crc_init(ref msg.checksum);
        }

        void mavlink_update_checksum(ref mavlink_message msg, uint8_t c){
            crc_accumulate(c, ref msg.checksum);
        }

        public byte mavlink_parse_char(uint8_t c) {
/*
	default message crc function. You can override this per-system to
	put this data in a different memory segment
*/

	       
	        int bufferIndex = 0;

	        status.msg_received = 0;

	        switch (status.parse_state) {
	   

	        case mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE:
		        if (c == MAVLINK_STX) {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_STX;
			        rxmsg.len = 0;
                    status.msg_error=0;
        //			rxmsg->magic = c;
			        mavlink_start_checksum(ref rxmsg);
        //mavlink_comm_0_port->printf_P(PSTR("\n\ngot STX!"));
                    return (byte)1;
		        } else {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
		        }
		        break;

	        case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_STX:
		        if (status.msg_received !=0
        /* Support shorter buffers than the
            default maximum packet size */
				        ){
			        status.buffer_overrun++;
			        status.parse_error++;
			        status.msg_received = 0;			        
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
		        }
		        else
		        {
        //mavlink_comm_0_port->printf_P(PSTR(" got Length!"));

			        // NOT counting STX, LENGTH, SEQ, SYSID, COMPID, MSGID, CRC1 and CRC2
			        rxmsg.len = c;
			        status.packet_idx = 0;
			        mavlink_update_checksum(ref rxmsg, c);
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_LENGTH;
		        }
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_LENGTH:
        //mavlink_comm_0_port->printf_P(PSTR(" got Seq!"));

		        rxmsg.seq = c;
		        mavlink_update_checksum(ref rxmsg, c);
                status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_SEQ;
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_SEQ:
        //mavlink_comm_0_port->printf_P(PSTR(" got Sysid!"));

		        rxmsg.sysid = c;
		        mavlink_update_checksum(ref rxmsg, c);
                status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_SYSID;
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_SYSID:
        //mavlink_comm_0_port->printf_P(PSTR(" got Compid!"));

		        rxmsg.compid = c;
		        mavlink_update_checksum(ref rxmsg, c);
                status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_COMPID;
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_COMPID:
		        rxmsg.msgid = c;
		        mavlink_update_checksum(ref rxmsg, c);
		        if (rxmsg.len == 0) {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_PAYLOAD;
		        } else {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_MSGID;
		        }               
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_MSGID:
                rxmsg.payload[status.packet_idx++] = (byte)c;
		        mavlink_update_checksum(ref rxmsg, c);
		        if (status.packet_idx == rxmsg.len) {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_PAYLOAD;

		        }                
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_PAYLOAD:
                mavlink_update_checksum(ref rxmsg, MAVLink.MAVLINK_MESSAGE_CRCS[rxmsg.msgid]);

		        if ( c != (rxmsg.checksum & 0xFF)) {
        Console.WriteLine("\n CRC1 err! want={0} got={1}", rxmsg.checksum & 0xFF, c);
			        // Check first checksum byte
			        status.parse_error++;
			        status.msg_received = 0;
                    status.msg_error = 1;
			        if (c == MAVLINK_STX) {
                        status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_STX;
                        rxmsg.len = 0;
                        status.msg_error = 0;
                        mavlink_start_checksum(ref rxmsg);

                    } else {
                        status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_BAD_CRC1;
			        }
		        } else {
 //  {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_CRC1;
                    rxmsg.payload[status.packet_idx] = (byte)c;
		        }
		        break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_BAD_CRC1:
                 if (c == MAVLINK_STX) {
                           status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_STX;
                           rxmsg.len = 0;
                           status.msg_error = 0;
                           mavlink_start_checksum(ref rxmsg);
                 } else {
                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
			        rxmsg.payload[status.packet_idx+1] = (byte)c;
                 }

                break;

            case mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_CRC1:
                //*
                   if (c != (rxmsg.checksum >> 8)) {// Check second checksum byte

                       Console.WriteLine("\nCRC2 err! want={0} got={1}", rxmsg.checksum >> 8, c);
                       status.msg_error = 1;
                        status.parse_error++;
                           status.msg_received = 0;
			
                           if (c == MAVLINK_STX) {
                                  status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_GOT_STX;
                                  rxmsg.len = 0;                                  
                                  mavlink_start_checksum(ref rxmsg);
                           } else {
                                        //status->parse_state = MAVLINK_PARSE_STATE_IDLE;     
                                   status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
                          }
                 } else 
                //*/ 
                       {
                  // Successfully got message
			        status.msg_received = 1;

                    status.parse_state = mavlink_parse_state_t.MAVLINK_PARSE_STATE_IDLE;
			        rxmsg.payload[status.packet_idx+1] = (byte)c;
                }
		        break;
	        }

	        bufferIndex++;
	        // If a message has been sucessfully decoded, check index
	        if (status.msg_received!=0) {
		        //while(status->current_seq != rxmsg->seq)
		        //{
		        //	status->packet_rx_drop_count++;
		        //               status->current_seq++;
		        //}
		        status.current_rx_seq = rxmsg.seq;
		        // Initial condition: If no packet has been received so far, drop count is undefined
		        if (status.packet_rx_success_count == 0) status.packet_rx_drop_count = 0;
		        // Count this packet as received
		        status.packet_rx_success_count++;
	        }

	        //status.parse_error = 0;
            status.current_rx_seq +=1;
	        //status.packet_rx_success_count = status->packet_rx_success_count;
	        status.packet_rx_drop_count = status.parse_error;
	        //r_mavlink_status->buffer_overrun = status->buffer_overrun;

            if(status.msg_error !=0) {
                status.msg_error=0;
                    return (byte)3;
            }
	        return status.msg_received!=0?(byte)2:(byte)0;
        }


        private void btnTLog_Click(object sender, EventArgs e) {
            

            if (!tlog_run) {
                CurrentCOM = CMB_ComPort.Text;
                tlog_thread = new System.Threading.Thread(thread_proc);
                tlog_run = true;
                comBusy=true ; 
                tlog_thread.Start();
            } else {
                need_stop_tlog=true;

                delay(100);
                Application.DoEvents();
                delay(100);

                if(tlog_run) {
                    tlog_run=false;
                    try {
                        tlog_thread.Abort();
                    } catch {};
                }
                try {
                    if (comPort.IsOpen)
                        comPort.Close();
                } catch {};
                comBusy = false;
            }
      
        }







        public string convertChars(string s){
            string so="";
            try {
                for(int i=0; i<s.Length; i++) {
                    char c = s[i];
                    if(c=='\\'){
                        i++;
                        if(i==s.Length) break;
                        switch(s[i]) {
                        case 'n':
                            so +="\n";
                            break;
                        case 'r':
                            so +="\r";
                            break;
                        case 't':
                            so +="\n";
                            break;
                            
                        case '0': // octal string
                        case '1': 
                            string oct="";
                            oct += s[i];
                            i++;
                            if(i==s.Length) break;
                            oct += s[i];
                            i++;
                            if(i==s.Length) break;
                            oct += s[i];
                            so += Convert.ToByte(oct,8);
                            break;
    
                        case 'x': // hex string
                            string hex="";
                            i++;
                            if(i==s.Length) break;
                            hex+=s[i];
                            i++;
                            if(i==s.Length) break;
                            hex+=s[i];
                            so += Convert.ToChar(Convert.ToByte(hex,16));
                            break;

                        }
                    } else {
                        so +=c;
                    }
                }
                return so;
            } catch{
                return s;
            }
        }

        public string myDecode(string s){
            string so="";
            try {
                for(int i=0; i<s.Length; i++) {
                    char c = s[i];
                    if(c==0) break;
                    if(c<0x20 || c>=0x80) {
                        string hex=Convert.ToString(c, 16);
                        if (hex.Length % 2 != 0)
                            hex = '0' + hex;
                        so+="\\x" + hex;
                    } else
                        so+=c;
                }
                return so;
            } catch{
                return s;
            }
        }

        float myConvert(string s) {
            bool f = false;
            double v = 0;
            
            
            do {
                try {
                    v = Convert.ToDouble(s);
                    f=true;
                } catch {                    
                };

                if(!f) {
                    string s1 = s.Replace('.', ',');
                    try {
                        v = Convert.ToDouble(s1);
                        f = true;
                    } catch {
                    };
                }
                if (!f && s.Length!=0) s = s.Substring(0, s.Length - 1);
            } while (!f && s != "");
            
            return (float)v;
        }

        //this.txtFactor1.TextChanged += new System.EventHandler(this.txtFactor1_TextChanged);
        private void txtFactor1_TextChanged(object sender, EventArgs e) {
            //if (((System.Windows.Forms.TextBox)sender).Focused) return;

            string s = ((System.Windows.Forms.TextBox)sender).Text;
            float v=myConvert(s);

            if(v==0) v=1;
            if(s != v.ToString())
                ((System.Windows.Forms.TextBox)sender).Text = v.ToString();
        }

        UInt64 get_timestamp(byte[] bytes, int ptr){
            UInt64 time=0;
            byte[] datearray=new Byte[8];
            for (int i = 0; i <8; i++) {
                datearray[i] = bytes[ptr + i];
            }
            Array.Reverse(datearray);
            time = (ulong)BitConverter.ToInt64(datearray,0);
            return time;
        }

        double millis(){
            return (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalMilliseconds;
        }


        void delay(int t){
            System.Threading.Thread.Sleep(t); 
        }

          
        private void parseInputData(string s) {
            Console.WriteLine(s);
        }

        [StructLayout(LayoutKind.Sequential, Pack = 1, Size = 136)]
        private struct Mav_conf { // needs to fit in 253 bytes
            public byte magick0; // = 0xee 'O' 'S' 'D'
            public byte magick1;
            public byte magick2;
            public byte magick3;
            public byte cmd;           // command
            public byte id;            // number of 128-bytes block
            public byte len;           // real length
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 128)]
            public byte[] data;
            public byte crc;           // may be
            [MarshalAs(UnmanagedType.ByValArray, SizeConst = 117)]
            public byte[] pad;
        };

        
        public static byte[] StructureToByteArray(object obj){ // Stolen from MissionPlanner
            int len = Marshal.SizeOf(obj);
            byte[] arr = new byte[len];
            IntPtr ptr = Marshal.AllocHGlobal(len);
            Marshal.StructureToPtr(obj, ptr, true);
            Marshal.Copy(ptr, arr, 0, len);
            Marshal.FreeHGlobal(ptr);
            return arr;
        }
        

        static T ByteArrayToStructureGC<T>(byte[] bytearray, int startoffset) where T : struct {
            GCHandle gch = GCHandle.Alloc(bytearray, GCHandleType.Pinned);
            try {
                return (T)Marshal.PtrToStructure(new IntPtr(gch.AddrOfPinnedObject().ToInt64() + startoffset), typeof(T));
            } finally {
                gch.Free();
            }
        }

        public void sendPacket(object indata) {
            bool validPacket = false;
            byte a = 0;
            foreach (Type ty in MAVLink.MAVLINK_MESSAGE_INFO) {
                if (ty == indata.GetType()) {
                    validPacket = true;
                    generatePacket(a, indata);
                    return;
                }
                a++;
            }
            if (!validPacket) {
                Console.WriteLine("Mavlink : NOT VALID PACKET sendPacket() " + indata.GetType().ToString());
            }
        }

        /// <summary>
        /// Generate a Mavlink Packet and write to serial
        /// </summary>
        /// <param name="messageType">type number = MAVLINK_MSG_ID</param>
        /// <param name="indata">struct of data</param>
        void generatePacket(byte messageType, object indata) {
           
            lock (objlock) {
                byte[] data;

                if(!flag_EEPROM_read) {
                    parseInputData(comPort.ReadExisting());
                }

                data = StructureToByteArray(indata);

                //Console.WriteLine(DateTime.Now + " PC Doing req "+ messageType + " " + this.BytesToRead);
                byte[] packet = new byte[data.Length + 6 + 2];
                int i = 0;
                packet[i++] = 254;
                packet[i++] = (byte)data.Length;
                packet[i++] = (byte)(MAV_packetcount++);
                packet[i++] = 255; // gcssysid; // this is always 255 - MYGCS
                packet[i++] = (byte)MAVLink.MAV_COMPONENT.MAV_COMP_ID_CAMERA;
                packet[i++] = messageType;
                
                foreach (byte b in data) {
                    packet[i++] = b;
                }

                ushort checksum = MAVLink.MavlinkCRC.crc_calculate(packet, packet[1] /* data.length */ + 6);

                checksum = MAVLink.MavlinkCRC.crc_accumulate(MAVLink.MAVLINK_MESSAGE_CRCS[messageType], checksum);

                byte ck_a = (byte)(checksum & 0xFF); ///< High byte
                byte ck_b = (byte)(checksum >> 8); ///< Low byte

                packet[i++] = ck_a;                
                packet[i++] = ck_b;                

                comPort.Write(packet, 0, i);

                delay(100); 
                if(!flag_EEPROM_read) {
                    parseInputData(comPort.ReadExisting());
                }
            }
        }

 

        private void Translate(Form f) {
            bool found = false;
            string fn = Path.GetDirectoryName(Application.ExecutablePath) + Path.DirectorySeparatorChar + "lang.txt";
            if (!File.Exists(fn)) {
                generateLang(fn);
                return;
            }

            using (StreamReader sr = new StreamReader(fn)) {
                while (!sr.EndOfStream) {

                    string s = sr.ReadLine();
                    if (!found) {
                        if (s == "[" + f.Name + "]")
                            found = true;
                        continue;
                    }
                    if (s[0] == '[') break; // next section
                    // found
                    string[] sa = s.Split('=');

                    string nm = sa[0].Substring(sa[0].Length - 3);
                    if (nm == "-tt") { // tooltip
                        Control ct = this.Controls.Find(sa[0].Substring(0, sa[0].Length - 3), true).FirstOrDefault() as Control;
                        if (ct != null)
                            hint.SetToolTip(ct, sa[1]);
                    } else {
                        Control c = this.Controls.Find(sa[0], true).FirstOrDefault() as Control;
                        if(c==null) continue;
                        if (c.ToString().Contains("System.Windows.Forms.TextBox")) continue;
                        if (c.ToString().Contains("System.Windows.Forms.ComboBox")) continue;
                        if (c.ToString().Contains("System.Windows.Forms.NumericUpDown")) continue;
                        if (c.ToString().Contains("System.Windows.Forms.MaskedTextBox")) continue;

                        c.Text = sa[1];
                    }
                }
            }
        }

        void parseControls(StreamWriter sw, Control.ControlCollection ctls) {
            foreach (Control c in ctls) {

                if (c.Controls != null && c.Controls.Count > 0) {
                    parseControls(sw, c.Controls);
                }

                if (c.Name == "") continue;
                if (c.Name == c.Text) continue;
                string tt = hint.GetToolTip(c);
                if (tt != "") {
                    sw.WriteLine(c.Name + "-tt" + "=" + tt);
                }

                if (c.Text == "") continue;
                //txtParam
                if (c.ToString().Contains("System.Windows.Forms.TextBox")) continue;
                if (c.ToString().Contains("System.Windows.Forms.ComboBox")) continue;
                if (c.ToString().Contains("System.Windows.Forms.NumericUpDown")) continue;
                if (c.ToString().Contains("System.Windows.Forms.MaskedTextBox")) continue;
                
                sw.WriteLine(c.Name + "=" + c.Text);
            }

            /*
                        foreach (System.Windows.Forms.ToolTip p in hint.tools) {

                        }
            */
        }

        void generateLang(string fn) {
            /*
                        List<Type> forms = new List<Type>();
                        foreach (Assembly asm in AppDomain.CurrentDomain.GetAssemblies()) {
                            forms.AddRange(from t in asm.GetTypes() where t.IsSubclassOf(typeof(Form)) select t);
                        }
                        foreach (object f in forms) {
                            Console.WriteLine ("f="+f.GetType());
                            string n = ((System.RuntimeType)f).
                        }
              */

            
            using (StreamWriter sw = new StreamWriter(fn)) {       //Write   
                sw.WriteLine("[" + this.Name + "]");

                parseControls(sw, this.Controls);

                sw.Close();
            }


        }

        private void Stand_Click(object sender, EventArgs e) {
        }

        private void comboBox1_Click(object sender, EventArgs e) {
            CMB_ComPort.Items.Clear();
            CMB_ComPort.Items.AddRange(GetPortNames());
        }

       

        


       

      
       

     


      

     
        

        

        

        
    


    }
}
