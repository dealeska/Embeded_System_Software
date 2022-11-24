using LiveChartsCore.Defaults;
using System;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Threading;

namespace STMReader
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            Task.Run(() => Logic.Start());
            Task.Run(() =>
            {
                while (true)
                {                    
                    if (Logic.PotentiometrBuffer.Count > 0)
                    {
                        var val = Logic.PotentiometrBuffer[0];
                        Dispatcher.BeginInvoke(new Action(() =>
                        {
                            if (Logic.PotentiometrValues.Count > 50)
                            {
                                Logic.PotentiometrValues.RemoveAt(0);
                            }
                            Logic.PotentiometrValues.Add(new ObservableValue(val));                            
                            
                        }));
                        Logic.PotentiometrBuffer.RemoveAt(0);
                    }

                    if (Logic.LightBuffer.Count > 0)
                    {
                        var val = Logic.LightBuffer[0];
                        Dispatcher.BeginInvoke(new Action(() =>
                        {
                            Logic.LightValues.Add(new ObservableValue(val));
                            if (Logic.LightValues.Count > 100)
                            {
                                Logic.LightValues.RemoveAt(0);
                            }
                        }));
                        Logic.LightBuffer.RemoveAt(0);
                    }                                       
                }
            });
        }

        private void SendRGB_b_Click(object sender, RoutedEventArgs e)
        {
            var rgb = new
            {
                red = Red_tb.Text,
                green = Green_tb.Text,
                blue = Blue_tb.Text
            };

            var message = $"RGB=r:{rgb.red},g:{rgb.green},b:{rgb.blue};";

            Logic.Send(message);
        }

        private void Red_tb_LostFocus(object sender, RoutedEventArgs e)
        {
            CheckConstraints(Red_tb);
        }

        private void Green_tb_LostFocus(object sender, RoutedEventArgs e)
        {
            CheckConstraints(Green_tb);
        }

        private void Blue_tb_LostFocus(object sender, RoutedEventArgs e)
        {
            CheckConstraints(Blue_tb);
        }

        private void CheckConstraints(TextBox textBox)
        {
            if (int.TryParse(textBox.Text, out int val))
            {
                if (val > 255)
                {
                    textBox.Text = "255";
                }
                if (val < 0)
                {
                    textBox.Text = "0";
                }
            }
            else
            {
                textBox.Text = "0";
            }
        }

        private void Pieso_tog_Click(object sender, RoutedEventArgs e)
        {
            if (Pieso_tog.IsChecked == true)
            {
                Logic.Send("PiesoIsOn");
            }
            else
            {
                Logic.Send("PiesoIsOff");
            }
        }
    }
}
