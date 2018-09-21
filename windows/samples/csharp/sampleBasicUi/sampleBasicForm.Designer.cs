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
            this.pbPreview = new System.Windows.Forms.PictureBox();
            this.tbLog = new System.Windows.Forms.TextBox();
            ((System.ComponentModel.ISupportInitialize)(this.pbPreview)).BeginInit();
            this.SuspendLayout();
            // 
            // pbPreview
            // 
            this.pbPreview.Location = new System.Drawing.Point(12, 12);
            this.pbPreview.Name = "pbPreview";
            this.pbPreview.Size = new System.Drawing.Size(456, 364);
            this.pbPreview.SizeMode = System.Windows.Forms.PictureBoxSizeMode.StretchImage;
            this.pbPreview.TabIndex = 1;
            this.pbPreview.TabStop = false;
            // 
            // tbLog
            // 
            this.tbLog.Location = new System.Drawing.Point(474, 13);
            this.tbLog.Multiline = true;
            this.tbLog.Name = "tbLog";
            this.tbLog.ReadOnly = true;
            this.tbLog.Size = new System.Drawing.Size(220, 363);
            this.tbLog.TabIndex = 2;
            // 
            // SampleBasicForm
            // 
            this.AutoScaleDimensions = new System.Drawing.SizeF(6F, 12F);
            this.AutoScaleMode = System.Windows.Forms.AutoScaleMode.Font;
            this.ClientSize = new System.Drawing.Size(706, 382);
            this.Controls.Add(this.tbLog);
            this.Controls.Add(this.pbPreview);
            this.Name = "SampleBasicForm";
            this.Text = "SampleBasicUi - SmartToF C#";
            ((System.ComponentModel.ISupportInitialize)(this.pbPreview)).EndInit();
            this.ResumeLayout(false);
            this.PerformLayout();

        }

        #endregion

        private System.Windows.Forms.PictureBox pbPreview;
        private System.Windows.Forms.TextBox tbLog;

    }
}

