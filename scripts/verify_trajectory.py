"""Read-only analysis regression for both CSV schemas; no UART."""
from pathlib import Path
import sys
import tempfile
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from web.server.flight_analysis import analyze_csv, FlightArchive

class AnalysisTest(unittest.TestCase):
    def test_target_rmse_and_invalid_points(self):
        text="actual_x,actual_y,actual_z,target_x,target_y,target_z,timestamp,segment,position_valid\n1,2,3,0,2,3,0,0,1\n4,2,3,0,2,3,1,0,0\n"
        result=analyze_csv(text)
        self.assertEqual(result['analysis']['rmse_3d_m'],1)
        self.assertEqual(result['analysis']['compared_points'],1)
        self.assertEqual(result['analysis']['invalid_points'],1)
    def test_historical_schema_and_breaks(self):
        result=analyze_csv("x_m,y_m,z_m,sample_ms,segment,yaw_deg,attitude_age_s\n0,0,-1,100,0,20,0.1\nnan,0,0,200,0,,\n0,0,-2,300,1,30,1\n")
        self.assertEqual(result['plotted_points'],2)
        self.assertTrue(result['points'][1]['break_before'])
        self.assertIsNone(result['points'][1]['yaw_deg'])
        self.assertIsNone(result['analysis']['rmse_3d_m'])
    def test_archive_only_complete(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for name in ['flight_1.csv','flight_2.partial.csv','flight_3.interrupted.csv']:
                (root/name).write_text('x_m,y_m,z_m\n0,0,0\n')
            archive=FlightArchive(root)
            self.assertEqual(archive.latest_file().name,'flight_1.csv')
            self.assertEqual(len(list(root.iterdir())),3)
    def test_invalid_header(self):
        with self.assertRaises(ValueError):analyze_csv('a,b\n1,2\n')

if __name__=='__main__':unittest.main()
