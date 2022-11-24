﻿using LiveChartsCore.Defaults;
using System;
using System.Collections.Generic;
using System.Collections.ObjectModel;
using System.IO.Ports;
using System.Text;
using System.Threading.Tasks;
using System.Globalization;

namespace STMReader
{
    internal class Logic
    {
        public static readonly ObservableCollection<ObservableValue> PotentiometrValues = new ObservableCollection<ObservableValue>();        
        public static readonly ObservableCollection<ObservableValue> LightValues = new ObservableCollection<ObservableValue>();
        public static List<float> PotentiometrBuffer = new List<float>();
        public static List<int> LightBuffer = new List<int>();

        static SerialPort ComPort = new SerialPort("COM4", 115200, Parity.None, 8, StopBits.One);

        [STAThread]
        public static void Start()
        {
            ComPort.DataReceived += new SerialDataReceivedEventHandler(port_DataReceived);
            ComPort.Open();
        }

        public static void Send(string message)
        {
            ComPort.Write(message);
        }         

        private static void port_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            Task.Run(() =>
            {
                //int readCount;
                //int bufReady = 0;
                //byte[] buf = new byte[128];
                //while (bufReady < buf.Length)
                //{
                //    readCount = ComPort.Read(buf, bufReady, buf.Length - bufReady);
                //    bufReady += readCount;
                //}

                
                //foreach (var val in values)
                //{
                    try
                    {
                        string raw = ComPort.ReadLine();
                        string data = Encoding.UTF8.GetString(Encoding.Default.GetBytes(raw));
                        //var values = data.Split("\n\r");

                        float potentiometrValue = 0f;
                        int lightValue = 0;
                        string value = data.Trim(new char[] { '\r', '\n' });

                        if (value.EndsWith(';'))
                        {
                            var index = value.IndexOf(',');
                            var end = value.IndexOf(';');

                            potentiometrValue = float.Parse(value.Substring(0, index), CultureInfo.InvariantCulture);
                            lightValue = int.Parse(value.Substring(index+1, end - index - 1), CultureInfo.InvariantCulture);

                            if (potentiometrValue <= 3.3f && potentiometrValue >= 0.0f)
                            {
                                PotentiometrBuffer.Add(potentiometrValue);
                            }
                            if (lightValue <= 100 && lightValue >= 0)
                            {
                                LightBuffer.Add(lightValue);
                            }
                        }
                    }
                    catch (Exception ex)
                    {
                        Console.WriteLine(ex.Message);
                    }
                //}
            });
        }
    }
}
