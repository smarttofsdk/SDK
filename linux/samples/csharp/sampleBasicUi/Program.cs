using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Forms;

namespace sampleBasicUi
{
    static class Program
    {
        /// <summary>
        /// 应用程序的主入口点。
        /// </summary>
        [STAThread]
        static void Main(string[] args)
        {
            Application.EnableVisualStyles();
            Application.SetCompatibleTextRenderingDefault(false);
            if (args.Length > 0 && args[0] == "-test")
            {
                Application.Run(new SampleBasicForm(1));
            }
            else
            {
                Application.Run(new SampleBasicForm());
            }
        }
    }
}
