using LiveChartsCore.Defaults;
using LiveChartsCore.SkiaSharpView.Painting;
using LiveChartsCore.SkiaSharpView;
using LiveChartsCore;
using SkiaSharp;

namespace STMReader
{
    internal class Init
    {
        public ISeries[] Potentiometr { get; set; } = new ISeries[]
        {
            new LineSeries<ObservableValue>
            {
                Values = Logic.PotentiometrValues,
                Name = "Potentiometr",
                Fill = null,
                Stroke = new SolidColorPaint(SKColors.Red, 2),
                GeometryStroke = new SolidColorPaint(SKColors.Red, 2),
                GeometrySize = 0,
                LineSmoothness = 0.5,
                DataLabelsSize= 0,
                /*DataLabelsPaint = new SolidColorPaint(SKColors.Blue),
                DataLabelsPosition = LiveChartsCore.Measure.DataLabelsPosition.Top,*/
            }            
        };

        public ISeries[] Lighting { get; set; } = new ISeries[]
        {
            new LineSeries<ObservableValue>
            {
                Values = Logic.LightValues,
                Name = "Lighting",
                Fill = null,
                Stroke = new SolidColorPaint(SKColors.Blue, 2),
                GeometryStroke = new SolidColorPaint(SKColors.Blue, 2),
                GeometrySize = 0,
                LineSmoothness = 0.5,
                DataLabelsSize= 0,
                /*DataLabelsPaint = new SolidColorPaint(SKColors.Red),
                DataLabelsPosition = LiveChartsCore.Measure.DataLabelsPosition.Top,*/
            }

        };
    }
}
