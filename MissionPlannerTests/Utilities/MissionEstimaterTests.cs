using Microsoft.VisualStudio.TestTools.UnitTesting;
using MissionPlanner.Utilities;
using System;
using System.Collections.Generic;

namespace MissionPlanner.Utilities.Tests
{
    [TestClass()]
    public class MissionEstimaterTests
    {
        [TestMethod()]
        public void TestCalculateTotal3DDistanceWithTerrain_TakeoffOnly()
        {
            // ホームポイント
            PointLatLngAlt home = new PointLatLngAlt(43.2894735, 143.2349412, 440.17, "HOME");

            // ウェイポイントリストにTAKEOFFのみ
            List<Locationwp> wpCommandList = new List<Locationwp>
            {
                new Locationwp
                {
                    lat = 0.0,
                    lng = 0.0,
                    alt = 15.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.TAKEOFF
                }
            };

            // 総距離を計算
            (double totalDistanceWithTerrain, double totalEstimatedFlightTime) = MissionEstimator.CalculateTotal3DDistanceWithTerrain(home, wpCommandList, false, 60, 10, 2.5, 2.5);

            // 期待値: TAKEOFFで15m上昇
            // 総距離 = 15m
            double expectedDistance = 15.0; // meters
            double expectedFlightTime = 6;
            double tolerance = 0.1; // meters

            // Assert.AreEqual を使用して期待値と実際の値を比較
            Assert.AreEqual(expectedDistance, Math.Round(totalDistanceWithTerrain, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalDistanceWithTerrain} meters, expected {expectedDistance} meters.");
            Assert.AreEqual(expectedFlightTime, Math.Round(totalEstimatedFlightTime, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalEstimatedFlightTime} seconds, expected {expectedFlightTime} seconds.");
        }

        [TestMethod()]
        public void TestCalculateTotal3DDistanceWithTerrain_TakeoffAndWaypoint()
        {
            // ホームポイント
            PointLatLngAlt home = new PointLatLngAlt(43.2894735, 143.2349412, 440.17, "HOME");

            // ウェイポイントリスト
            List<Locationwp> wpCommandList = new List<Locationwp>
            {
                new Locationwp
                {
                    lat = 43.2894735,
                    lng = 143.2349412,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.TAKEOFF
                },
                new Locationwp
                {
                    lat = 43.3003966,
                    lng = 143.254574,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.WAYPOINT
                }
            };

            // 総距離を計算
            (double totalDistanceWithTerrain, double totalEstimatedFlightTime) = MissionEstimator.CalculateTotal3DDistanceWithTerrain(home, wpCommandList, false, 60, 10, 2.5, 2.5);

            // 期待値: TAKEOFFで10m上昇、次のウェイポイントで10m移動
            // 総距離 = 10m (高度差) + 2000m (水平移動) = 2010m
            double expectedDistance = 2010.0; // meters
            double expectedFlightTime = 204;
            double tolerance = 0.1; // meters

            // Assert.AreEqual を使用して期待値と実際の値を比較
            Assert.AreEqual(expectedDistance, Math.Round(totalDistanceWithTerrain, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalDistanceWithTerrain} meters, expected {expectedDistance} meters.");
            Assert.AreEqual(expectedFlightTime, Math.Round(totalEstimatedFlightTime, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalEstimatedFlightTime} seconds, expected {expectedFlightTime} seconds.");
        }

        [TestMethod()]
        public void TestCalculateTotal3DDistance_MultipleWaypoints()
        {
            // ホームポイント
            PointLatLngAlt home = new PointLatLngAlt(43.2894735, 143.2349412, 440.17, "HOME");

            // ウェイポイントリスト
            List<Locationwp> wpCommandList = new List<Locationwp>
            {
                new Locationwp
                {
                    lat = 0.0,
                    lng = 0.0,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.TAKEOFF
                },
                new Locationwp
                {
                    lat = 43.3003966,
                    lng = 143.254574,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.WAYPOINT
                },
                new Locationwp
                {
                    lat = 43.3007270,
                    lng = 143.2582530,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.WAYPOINT
                }
            };

            // 総距離を計算
            (double totalDistanceWithTerrain, double totalEstimatedFlightTime) = MissionEstimator.CalculateTotal3DDistanceWithTerrain(home, wpCommandList, false, 60, 10, 2.5, 2.5);

            // 期待値: TAKEOFFで10m上昇、次のウェイポイントで10m移動、さらにもう一つのウェイポイントで10m移動
            // 総距離 = 10m (TAKEOFF) + 2000m (最初のウェイポイント移動) + 300m (次のウェイポイント移動) = 2310m
            double expectedDistance = 2310.0; // meters
            double expectedFlightTime = 234;
            double tolerance = 0.1; // meters

            // Assert.AreEqual を使用して期待値と実際の値を比較
            Assert.AreEqual(expectedDistance, Math.Round(totalDistanceWithTerrain, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalDistanceWithTerrain} meters, expected {expectedDistance} meters.");
            Assert.AreEqual(expectedFlightTime, Math.Round(totalEstimatedFlightTime, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalEstimatedFlightTime} seconds, expected {expectedFlightTime} seconds.");
        }
        [TestMethod()]
        public void TestCalculateTotal3DDistance_RTL()
        {
            // ホームポイント
            PointLatLngAlt home = new PointLatLngAlt(43.2894735, 143.2349412, 440.17, "HOME");

            // ウェイポイントリスト
            List<Locationwp> wpCommandList = new List<Locationwp>
            {
                new Locationwp
                {
                    lat = 0.0,
                    lng = 0.0,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.TAKEOFF
                },
                new Locationwp
                {
                    lat = 43.3003966,
                    lng = 143.254574,
                    alt = 10.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.WAYPOINT
                },
                new Locationwp
                {
                    lat = 0.0,
                    lng = 0.0,
                    alt = 0.0f,
                    frame = (int)MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT,
                    id = (int)MAVLink.MAV_CMD.RETURN_TO_LAUNCH
                }
            };

            // 総距離を計算
            (double totalDistanceWithTerrain, double totalEstimatedFlightTime) = MissionEstimator.CalculateTotal3DDistanceWithTerrain(home, wpCommandList, false, 60, 10, 2.5, 2.5);

            // 期待値: TAKEOFFで10m上昇、次のウェイポイントで10m移動、さらにもう一つのウェイポイントで10m移動
            // 総距離 = 10m (TAKEOFF) + 2000m (最初のウェイポイント移動) + 50m (上昇) + 2000m + 60m (Land) = 4120m
            double expectedDistance = 4120.0; // meters
            double expectedFlightTime = 448;
            double tolerance = 0.1; // meters

            // Assert.AreEqual を使用して期待値と実際の値を比較
            Assert.AreEqual(expectedDistance, Math.Round(totalDistanceWithTerrain, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalDistanceWithTerrain} meters, expected {expectedDistance} meters.");
            Assert.AreEqual(expectedFlightTime, Math.Round(totalEstimatedFlightTime, 2), tolerance, $"CalculateTotal3DDistanceWithTerrain returned {totalEstimatedFlightTime} seconds, expected {expectedFlightTime} seconds.");
        }
    }
}
