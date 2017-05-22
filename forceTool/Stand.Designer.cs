using System;
namespace OSD {
    partial class Stand {
        /// <summary>
        /// Required designer variable.
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// Clean up any resources being used.
        /// </summary>
        /// <param name="disposing">true if managed resources should be disposed; otherwise, false.</param>
        protected override void Dispose(bool disposing) {
            if (disposing && (components != null)) {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows Form Designer generated code

        /// <summary>
        /// Required method for Designer support - do not modify
        /// the contents of this method with the code editor.
        /// </summary>
        private void InitializeComponent() {
            this.components = new System.ComponentModel.Container();
            System.ComponentModel.ComponentResourceManager resources = new System.ComponentModel.ComponentResourceManager(typeof(Stand));
            this.CMB_ComPort = new System.Windows.Forms.ComboBox();
            this.label5 = new System.Windows.Forms.Label();
            this.cbxAutoUpdate = new System.Windows.Forms.CheckBox();
            this.cbxShowUpdateDialog = new System.Windows.Forms.CheckBox();
            this.hint = new System.Windows.Forms.ToolTip(this.components);
            this.label1 = new System.Windows.Forms.Label();
            this.txtMin = new System.Windows.Forms.TextBox();
            this.txtMax = new System.Windows.Forms.TextBox();
            this.label2 = new System.Windows.Forms.Label();
            this.txtStep = new System.Windows.Forms.TextBox();
            this.label3 = new System.Windows.Forms.Label();
            this.btnStart = new System.Windows.Forms.Button();
            this.txtPWM = new System.Windows.Forms.TextBox();
            this.label4 = new System.Windows.Forms.Label();
            this.txtStepTime = new System.Windows.Forms.TextBox();
            this.label6 = new System.Windows.Forms.Label();
            this.txtName = new System.Windows.Forms.TextBox();
            this.label7 = new System.Windows.Forms.Label();
            this.btnCalib = new System.Windows.Forms.Button();
            this.tabControl1 = new System.Windows.Forms.TabControl();
            this.tabPage1 = new System.Windows.Forms.TabPage();
            this.tabPage2 = new System.Windows.Forms.TabPage();
            this.btnSend = new System.Windows.Forms.Button();
            this.lstConsole = new System.Windows.Forms.ListBox();
            this.txtCmd = new System.Windows.Forms.TextBox();
            this.label15 = new System.Windows.Forms.Label();
            this.txtData6 = new System.Windows.Forms.TextBox();
            this.label14 = new System.Windows.Forms.Label();
            this.txtData5 = new System.Windows.Forms.TextBox();
            this.label13 = new System.Windows.Forms.Label();
            this.txtData4 = new System.Windows.Forms.TextBox();
            this.label12 = new System.Windows.Forms.Label();
            this.txtData3 = new System.Windows.Forms.TextBox();
            this.label11 = new System.Windows.Forms.Label();
            this.txtData2 = new System.Windows.Forms.TextBox();
            this.label10 = new System.Windows.Forms.Label();
            this.txtData1 = new System.Windows.Forms.TextBox();
            this.label9 = new System.Windows.Forms.Label();
            this.txtData0 = new System.Windows.Forms.TextBox();
            this.label8 = new System.Windows.Forms.Label();
            this.picData0 = new System.Windows.Forms.PictureBox();
            this.picData1 = new System.Windows.Forms.PictureBox();
            this.picData2 = new System.Windows.Forms.PictureBox();
            this.picData3 = new System.Windows.Forms.PictureBox();
            this.picData4 = new System.Windows.Forms.PictureBox();
            this.picData5 = new System.Windows.Forms.PictureBox();
            this.tabControl1.SuspendLayout();
            this.tabPage1.SuspendLayout();
            this.tabPage2.SuspendLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picData0)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData1)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData2)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData3)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData4)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData5)).BeginInit();
            this.SuspendLayout();
            // 
            // CMB_ComPort
            // 
            this.CMB_ComPort.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.CMB_ComPort.FormattingEnabled = true;
            this.CMB_ComPort.Location = new System.Drawing.Point(654, -199);
            this.CMB_ComPort.Name = "CMB_ComPort";
            this.CMB_ComPort.Size = new System.Drawing.Size(98, 21);
            this.CMB_ComPort.TabIndex = 4;
            // 
            // label5
            // 
            this.label5.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Bottom | System.Windows.Forms.AnchorStyles.Right)));
            this.label5.AutoSize = true;
            this.label5.Location = new System.Drawing.Point(590, -196);
            this.label5.Name = "label5";
            this.label5.Size = new System.Drawing.Size(58, 13);
            this.label5.TabIndex = 16;
            this.label5.Text = "Serial Port:";
            // 
            // cbxAutoUpdate
            // 
            this.cbxAutoUpdate.AutoSize = true;
            this.cbxAutoUpdate.Checked = true;
            this.cbxAutoUpdate.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxAutoUpdate.Location = new System.Drawing.Point(7, 6);
            this.cbxAutoUpdate.Name = "cbxAutoUpdate";
            this.cbxAutoUpdate.Size = new System.Drawing.Size(160, 17);
            this.cbxAutoUpdate.TabIndex = 0;
            this.cbxAutoUpdate.Text = "Check for updates at startup";
            this.cbxAutoUpdate.UseVisualStyleBackColor = true;
            // 
            // cbxShowUpdateDialog
            // 
            this.cbxShowUpdateDialog.AutoSize = true;
            this.cbxShowUpdateDialog.Checked = true;
            this.cbxShowUpdateDialog.CheckState = System.Windows.Forms.CheckState.Checked;
            this.cbxShowUpdateDialog.Location = new System.Drawing.Point(7, 29);
            this.cbxShowUpdateDialog.Name = "cbxShowUpdateDialog";
            this.cbxShowUpdateDialog.Size = new System.Drawing.Size(157, 17);
            this.cbxShowUpdateDialog.TabIndex = 1;
            this.cbxShowUpdateDialog.Text = "Prompt for update at startup";
            this.cbxShowUpdateDialog.UseVisualStyleBackColor = true;
            // 
            // label1
            // 
            this.label1.AutoSize = true;
            this.label1.Location = new System.Drawing.Point(27, 42);
            this.label1.Name = "label1";
            this.label1.Size = new System.Drawing.Size(24, 13);
            this.label1.TabIndex = 45;
            this.label1.Text = "Min";
            // 
            // txtMin
            // 
            this.txtMin.Location = new System.Drawing.Point(30, 58);
            this.txtMin.Name = "txtMin";
            this.txtMin.Size = new System.Drawing.Size(73, 20);
            this.txtMin.TabIndex = 46;
            this.txtMin.Text = "1100";
            // 
            // txtMax
            // 
            this.txtMax.Location = new System.Drawing.Point(109, 58);
            this.txtMax.Name = "txtMax";
            this.txtMax.Size = new System.Drawing.Size(73, 20);
            this.txtMax.TabIndex = 48;
            this.txtMax.Text = "1900";
            // 
            // label2
            // 
            this.label2.AutoSize = true;
            this.label2.Location = new System.Drawing.Point(106, 42);
            this.label2.Name = "label2";
            this.label2.Size = new System.Drawing.Size(27, 13);
            this.label2.TabIndex = 47;
            this.label2.Text = "Max";
            // 
            // txtStep
            // 
            this.txtStep.Location = new System.Drawing.Point(188, 58);
            this.txtStep.Name = "txtStep";
            this.txtStep.Size = new System.Drawing.Size(73, 20);
            this.txtStep.TabIndex = 50;
            this.txtStep.Text = "50";
            // 
            // label3
            // 
            this.label3.AutoSize = true;
            this.label3.Location = new System.Drawing.Point(185, 42);
            this.label3.Name = "label3";
            this.label3.Size = new System.Drawing.Size(27, 13);
            this.label3.TabIndex = 49;
            this.label3.Text = "step";
            // 
            // btnStart
            // 
            this.btnStart.Location = new System.Drawing.Point(357, 53);
            this.btnStart.Name = "btnStart";
            this.btnStart.Size = new System.Drawing.Size(76, 28);
            this.btnStart.TabIndex = 51;
            this.btnStart.Text = "Start";
            this.btnStart.UseVisualStyleBackColor = true;
            this.btnStart.Click += new System.EventHandler(this.btnStart_Click);
            // 
            // txtPWM
            // 
            this.txtPWM.Location = new System.Drawing.Point(455, 58);
            this.txtPWM.Name = "txtPWM";
            this.txtPWM.Size = new System.Drawing.Size(73, 20);
            this.txtPWM.TabIndex = 53;
            // 
            // label4
            // 
            this.label4.AutoSize = true;
            this.label4.Location = new System.Drawing.Point(452, 42);
            this.label4.Name = "label4";
            this.label4.Size = new System.Drawing.Size(34, 13);
            this.label4.TabIndex = 52;
            this.label4.Text = "PWM";
            // 
            // txtStepTime
            // 
            this.txtStepTime.Location = new System.Drawing.Point(267, 58);
            this.txtStepTime.Name = "txtStepTime";
            this.txtStepTime.Size = new System.Drawing.Size(73, 20);
            this.txtStepTime.TabIndex = 55;
            this.txtStepTime.Text = "500";
            // 
            // label6
            // 
            this.label6.AutoSize = true;
            this.label6.Location = new System.Drawing.Point(264, 42);
            this.label6.Name = "label6";
            this.label6.Size = new System.Drawing.Size(68, 13);
            this.label6.TabIndex = 54;
            this.label6.Text = "step time, ms";
            // 
            // txtName
            // 
            this.txtName.Location = new System.Drawing.Point(68, 11);
            this.txtName.Name = "txtName";
            this.txtName.Size = new System.Drawing.Size(384, 20);
            this.txtName.TabIndex = 57;
            this.txtName.Text = "test";
            // 
            // label7
            // 
            this.label7.AutoSize = true;
            this.label7.Location = new System.Drawing.Point(27, 14);
            this.label7.Name = "label7";
            this.label7.Size = new System.Drawing.Size(35, 13);
            this.label7.TabIndex = 56;
            this.label7.Text = "Name";
            // 
            // btnCalib
            // 
            this.btnCalib.Location = new System.Drawing.Point(546, 59);
            this.btnCalib.Name = "btnCalib";
            this.btnCalib.Size = new System.Drawing.Size(97, 22);
            this.btnCalib.TabIndex = 72;
            this.btnCalib.Text = "Recalibrate";
            this.btnCalib.UseVisualStyleBackColor = true;
            this.btnCalib.Click += new System.EventHandler(this.btnCalib_Click);
            // 
            // tabControl1
            // 
            this.tabControl1.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.tabControl1.Controls.Add(this.tabPage1);
            this.tabControl1.Controls.Add(this.tabPage2);
            this.tabControl1.Location = new System.Drawing.Point(3, 162);
            this.tabControl1.Name = "tabControl1";
            this.tabControl1.SelectedIndex = 0;
            this.tabControl1.Size = new System.Drawing.Size(983, 257);
            this.tabControl1.TabIndex = 77;
            // 
            // tabPage1
            // 
            this.tabPage1.Controls.Add(this.picData5);
            this.tabPage1.Controls.Add(this.picData4);
            this.tabPage1.Controls.Add(this.picData3);
            this.tabPage1.Controls.Add(this.picData2);
            this.tabPage1.Controls.Add(this.picData1);
            this.tabPage1.Controls.Add(this.picData0);
            this.tabPage1.Location = new System.Drawing.Point(4, 22);
            this.tabPage1.Name = "tabPage1";
            this.tabPage1.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage1.Size = new System.Drawing.Size(975, 231);
            this.tabPage1.TabIndex = 0;
            this.tabPage1.Text = "Data";
            this.tabPage1.UseVisualStyleBackColor = true;
            // 
            // tabPage2
            // 
            this.tabPage2.Controls.Add(this.btnSend);
            this.tabPage2.Controls.Add(this.lstConsole);
            this.tabPage2.Controls.Add(this.txtCmd);
            this.tabPage2.Controls.Add(this.label15);
            this.tabPage2.Location = new System.Drawing.Point(4, 22);
            this.tabPage2.Name = "tabPage2";
            this.tabPage2.Padding = new System.Windows.Forms.Padding(3);
            this.tabPage2.Size = new System.Drawing.Size(956, 209);
            this.tabPage2.TabIndex = 1;
            this.tabPage2.Text = "Console";
            this.tabPage2.UseVisualStyleBackColor = true;
            // 
            // btnSend
            // 
            this.btnSend.Anchor = ((System.Windows.Forms.AnchorStyles)((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Right)));
            this.btnSend.Location = new System.Drawing.Point(882, 15);
            this.btnSend.Name = "btnSend";
            this.btnSend.Size = new System.Drawing.Size(68, 20);
            this.btnSend.TabIndex = 88;
            this.btnSend.Text = "Send";
            this.btnSend.UseVisualStyleBackColor = true;
            this.btnSend.Click += new System.EventHandler(this.btnSend_Click);
            // 
            // lstConsole
            // 
            this.lstConsole.Anchor = ((System.Windows.Forms.AnchorStyles)((((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Bottom) 
            | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.lstConsole.FormattingEnabled = true;
            this.lstConsole.Location = new System.Drawing.Point(6, 41);
            this.lstConsole.Name = "lstConsole";
            this.lstConsole.Size = new System.Drawing.Size(944, 160);
            this.lstConsole.TabIndex = 87;
            // 
            // txtCmd
            // 
            this.txtCmd.Anchor = ((System.Windows.Forms.AnchorStyles)(((System.Windows.Forms.AnchorStyles.Top | System.Windows.Forms.AnchorStyles.Left) 
            | System.Windows.Forms.AnchorStyles.Right)));
            this.txtCmd.Location = new System.Drawing.Point(57, 15);
            this.txtCmd.Name = "txtCmd";
            this.txtCmd.Size = new System.Drawing.Size(819, 20);
            this.txtCmd.TabIndex = 86;
            // 
            // label15
            // 
            this.label15.AutoSize = true;
            this.label15.Location = new System.Drawing.Point(11, 18);
            this.label15.Name = "label15";
            this.label15.Size = new System.Drawing.Size(45, 13);
            this.label15.TabIndex = 85;
            this.label15.Text = "Console";
            // 
            // txtData6
            // 
            this.txtData6.Location = new System.Drawing.Point(504, 122);
            this.txtData6.Name = "txtData6";
            this.txtData6.Size = new System.Drawing.Size(73, 20);
            this.txtData6.TabIndex = 99;
            this.txtData6.Text = "0";
            // 
            // label14
            // 
            this.label14.AutoSize = true;
            this.label14.Location = new System.Drawing.Point(501, 106);
            this.label14.Name = "label14";
            this.label14.Size = new System.Drawing.Size(40, 13);
            this.label14.TabIndex = 98;
            this.label14.Text = "data[6]";
            // 
            // txtData5
            // 
            this.txtData5.Location = new System.Drawing.Point(425, 122);
            this.txtData5.Name = "txtData5";
            this.txtData5.Size = new System.Drawing.Size(73, 20);
            this.txtData5.TabIndex = 97;
            this.txtData5.Text = "0";
            // 
            // label13
            // 
            this.label13.AutoSize = true;
            this.label13.Location = new System.Drawing.Point(422, 106);
            this.label13.Name = "label13";
            this.label13.Size = new System.Drawing.Size(40, 13);
            this.label13.TabIndex = 96;
            this.label13.Text = "data[5]";
            // 
            // txtData4
            // 
            this.txtData4.Location = new System.Drawing.Point(346, 122);
            this.txtData4.Name = "txtData4";
            this.txtData4.Size = new System.Drawing.Size(73, 20);
            this.txtData4.TabIndex = 95;
            this.txtData4.Text = "0";
            // 
            // label12
            // 
            this.label12.AutoSize = true;
            this.label12.Location = new System.Drawing.Point(343, 106);
            this.label12.Name = "label12";
            this.label12.Size = new System.Drawing.Size(40, 13);
            this.label12.TabIndex = 94;
            this.label12.Text = "data[4]";
            // 
            // txtData3
            // 
            this.txtData3.Location = new System.Drawing.Point(267, 122);
            this.txtData3.Name = "txtData3";
            this.txtData3.Size = new System.Drawing.Size(73, 20);
            this.txtData3.TabIndex = 93;
            this.txtData3.Text = "0";
            // 
            // label11
            // 
            this.label11.AutoSize = true;
            this.label11.Location = new System.Drawing.Point(264, 106);
            this.label11.Name = "label11";
            this.label11.Size = new System.Drawing.Size(40, 13);
            this.label11.TabIndex = 92;
            this.label11.Text = "data[3]";
            // 
            // txtData2
            // 
            this.txtData2.Location = new System.Drawing.Point(188, 122);
            this.txtData2.Name = "txtData2";
            this.txtData2.Size = new System.Drawing.Size(73, 20);
            this.txtData2.TabIndex = 91;
            this.txtData2.Text = "0";
            // 
            // label10
            // 
            this.label10.AutoSize = true;
            this.label10.Location = new System.Drawing.Point(185, 106);
            this.label10.Name = "label10";
            this.label10.Size = new System.Drawing.Size(40, 13);
            this.label10.TabIndex = 90;
            this.label10.Text = "data[2]";
            // 
            // txtData1
            // 
            this.txtData1.Location = new System.Drawing.Point(109, 122);
            this.txtData1.Name = "txtData1";
            this.txtData1.Size = new System.Drawing.Size(73, 20);
            this.txtData1.TabIndex = 89;
            this.txtData1.Text = "0";
            // 
            // label9
            // 
            this.label9.AutoSize = true;
            this.label9.Location = new System.Drawing.Point(106, 106);
            this.label9.Name = "label9";
            this.label9.Size = new System.Drawing.Size(40, 13);
            this.label9.TabIndex = 88;
            this.label9.Text = "data[1]";
            // 
            // txtData0
            // 
            this.txtData0.Location = new System.Drawing.Point(30, 122);
            this.txtData0.Name = "txtData0";
            this.txtData0.Size = new System.Drawing.Size(73, 20);
            this.txtData0.TabIndex = 87;
            this.txtData0.Text = "0";
            // 
            // label8
            // 
            this.label8.AutoSize = true;
            this.label8.Location = new System.Drawing.Point(27, 108);
            this.label8.Name = "label8";
            this.label8.Size = new System.Drawing.Size(40, 13);
            this.label8.TabIndex = 86;
            this.label8.Text = "data[0]";
            // 
            // picData0
            // 
            this.picData0.Location = new System.Drawing.Point(11, 10);
            this.picData0.Name = "picData0";
            this.picData0.Size = new System.Drawing.Size(300, 100);
            this.picData0.TabIndex = 0;
            this.picData0.TabStop = false;
            // 
            // picData1
            // 
            this.picData1.Location = new System.Drawing.Point(317, 10);
            this.picData1.Name = "picData1";
            this.picData1.Size = new System.Drawing.Size(300, 100);
            this.picData1.TabIndex = 1;
            this.picData1.TabStop = false;
            // 
            // picData2
            // 
            this.picData2.Location = new System.Drawing.Point(623, 10);
            this.picData2.Name = "picData2";
            this.picData2.Size = new System.Drawing.Size(300, 100);
            this.picData2.TabIndex = 2;
            this.picData2.TabStop = false;
            // 
            // picData3
            // 
            this.picData3.Location = new System.Drawing.Point(11, 116);
            this.picData3.Name = "picData3";
            this.picData3.Size = new System.Drawing.Size(300, 100);
            this.picData3.TabIndex = 3;
            this.picData3.TabStop = false;
            // 
            // picData4
            // 
            this.picData4.Location = new System.Drawing.Point(317, 116);
            this.picData4.Name = "picData4";
            this.picData4.Size = new System.Drawing.Size(300, 100);
            this.picData4.TabIndex = 4;
            this.picData4.TabStop = false;
            // 
            // picData5
            // 
            this.picData5.Location = new System.Drawing.Point(623, 116);
            this.picData5.Name = "picData5";
            this.picData5.Size = new System.Drawing.Size(300, 100);
            this.picData5.TabIndex = 5;
            this.picData5.TabStop = false;
            // 
            // Stand
            // 
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.None;
            this.ClientSize = new System.Drawing.Size(988, 421);
            this.Controls.Add(this.txtData6);
            this.Controls.Add(this.label14);
            this.Controls.Add(this.txtData5);
            this.Controls.Add(this.label13);
            this.Controls.Add(this.txtData4);
            this.Controls.Add(this.label12);
            this.Controls.Add(this.txtData3);
            this.Controls.Add(this.label11);
            this.Controls.Add(this.txtData2);
            this.Controls.Add(this.label10);
            this.Controls.Add(this.txtData1);
            this.Controls.Add(this.label9);
            this.Controls.Add(this.txtData0);
            this.Controls.Add(this.label8);
            this.Controls.Add(this.tabControl1);
            this.Controls.Add(this.btnCalib);
            this.Controls.Add(this.txtName);
            this.Controls.Add(this.label7);
            this.Controls.Add(this.txtStepTime);
            this.Controls.Add(this.label6);
            this.Controls.Add(this.txtPWM);
            this.Controls.Add(this.label4);
            this.Controls.Add(this.btnStart);
            this.Controls.Add(this.txtStep);
            this.Controls.Add(this.label3);
            this.Controls.Add(this.txtMax);
            this.Controls.Add(this.label2);
            this.Controls.Add(this.txtMin);
            this.Controls.Add(this.label1);
            this.Controls.Add(this.label5);
            this.Controls.Add(this.CMB_ComPort);
            this.Icon = ((System.Drawing.Icon)(resources.GetObject("$this.Icon")));
            this.Name = "Stand";
            this.Text = "Prop stand tool";
            this.FormClosed += new System.Windows.Forms.FormClosedEventHandler(this.OSD_FormClosed);
            this.Load += new System.EventHandler(this.OSD_Load);
            this.Click += new System.EventHandler(this.Stand_Click);
            this.tabControl1.ResumeLayout(false);
            this.tabPage1.ResumeLayout(false);
            this.tabPage2.ResumeLayout(false);
            this.tabPage2.PerformLayout();
            ((System.ComponentModel.ISupportInitialize)(this.picData0)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData1)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData2)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData3)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData4)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.picData5)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        public System.Windows.Forms.ComboBox CMB_ComPort;
        //        private System.Windows.Forms.RadioButton rbtBatterymAh;
        //        private System.Windows.Forms.RadioButton rbtBatteryPercent;
        private System.Windows.Forms.Label label5;
        private System.Windows.Forms.CheckBox cbxAutoUpdate;
        private System.Windows.Forms.CheckBox cbxShowUpdateDialog;
        private System.Windows.Forms.ToolTip hint;
        private System.Windows.Forms.Label label1;
        private System.Windows.Forms.TextBox txtMin;
        private System.Windows.Forms.TextBox txtMax;
        private System.Windows.Forms.Label label2;
        private System.Windows.Forms.TextBox txtStep;
        private System.Windows.Forms.Label label3;
        private System.Windows.Forms.Button btnStart;
        private System.Windows.Forms.TextBox txtPWM;
        private System.Windows.Forms.Label label4;
        private System.Windows.Forms.TextBox txtStepTime;
        private System.Windows.Forms.Label label6;
        private System.Windows.Forms.TextBox txtName;
        private System.Windows.Forms.Label label7;
        private System.Windows.Forms.Button btnCalib;
        private System.Windows.Forms.TabControl tabControl1;
        private System.Windows.Forms.TabPage tabPage1;
        private System.Windows.Forms.TabPage tabPage2;
        private System.Windows.Forms.Button btnSend;
        private System.Windows.Forms.ListBox lstConsole;
        private System.Windows.Forms.TextBox txtCmd;
        private System.Windows.Forms.Label label15;
        private System.Windows.Forms.TextBox txtData6;
        private System.Windows.Forms.Label label14;
        private System.Windows.Forms.TextBox txtData5;
        private System.Windows.Forms.Label label13;
        private System.Windows.Forms.TextBox txtData4;
        private System.Windows.Forms.Label label12;
        private System.Windows.Forms.TextBox txtData3;
        private System.Windows.Forms.Label label11;
        private System.Windows.Forms.TextBox txtData2;
        private System.Windows.Forms.Label label10;
        private System.Windows.Forms.TextBox txtData1;
        private System.Windows.Forms.Label label9;
        private System.Windows.Forms.TextBox txtData0;
        private System.Windows.Forms.Label label8;
        private System.Windows.Forms.PictureBox picData5;
        private System.Windows.Forms.PictureBox picData4;
        private System.Windows.Forms.PictureBox picData3;
        private System.Windows.Forms.PictureBox picData2;
        private System.Windows.Forms.PictureBox picData1;
        private System.Windows.Forms.PictureBox picData0;
    }

}

