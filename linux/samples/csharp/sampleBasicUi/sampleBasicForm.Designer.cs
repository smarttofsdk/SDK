namespace sampleBasicUi
{
    partial class SampleBasicForm
    {
        /// <summary>
        /// 必需的设计器变量。
        /// </summary>
        private System.ComponentModel.IContainer components = null;

        /// <summary>
        /// 清理所有正在使用的资源。
        /// </summary>
        /// <param name="disposing">如果应释放托管资源，为 true；否则为 false。</param>
        protected override void Dispose(bool disposing)
        {
            if (disposing && (components != null))
            {
                components.Dispose();
            }
            base.Dispose(disposing);
        }

        #region Windows 窗体设计器生成的代码

        /// <summary>
        /// 设计器支持所需的方法 - 不要
        /// 使用代码编辑器修改此方法的内容。
        /// </summary>
        private void InitializeComponent()
        {
            this.pbDepth = new System.Windows.Forms.PictureBox();
            this.tbLog = new System.Windows.Forms.TextBox();
            this.pbIR = new System.Windows.Forms.PictureBox();
            ((System.ComponentModel.ISupportInitialize)(this.pbDepth)).BeginInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbIR)).BeginInit();
            this.SuspendLayout();
            // 
            // pbDepth
            // 
            this.pbDepth.Location = new System.Drawing.Point(12, 12);
            this.pbDepth.Name = "pbDepth";
            this.pbDepth.Size = new System.Drawing.Size(320, 240);
            this.pbDepth.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pbDepth.TabIndex = 1;
            this.pbDepth.TabStop = false;
            // 
            // tbLog
            // 
            this.tbLog.Location = new System.Drawing.Point(338, 12);
            this.tbLog.Multiline = true;
            this.tbLog.Name = "tbLog";
            this.tbLog.ReadOnly = true;
            this.tbLog.Size = new System.Drawing.Size(184, 486);
            this.tbLog.TabIndex = 2;
            // 
            // pbIR
            // 
            this.pbIR.Location = new System.Drawing.Point(12, 258);
            this.pbIR.Name = "pbIR";
            this.pbIR.Size = new System.Drawing.Size(320, 240);
            this.pbIR.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pbIR.TabIndex = 1;
            this.pbIR.TabStop = false;
            // 
            // SampleBasicForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(526, 502);
            this.Controls.Add(this.tbLog);
            this.Controls.Add(this.pbIR);
            this.Controls.Add(this.pbDepth);
            this.Name = "SampleBasicForm";
            this.Text = "SampleBasicUi - SmartToF C#";
            ((System.ComponentModel.ISupportInitialize)(this.pbDepth)).EndInit();
            ((System.ComponentModel.ISupportInitialize)(this.pbIR)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox pbDepth;
        private System.Windows.Forms.TextBox tbLog;
        private System.Windows.Forms.PictureBox pbIR;

    }
}

