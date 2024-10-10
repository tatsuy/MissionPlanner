﻿using System;
using static MAVLink;
using System.Collections.Generic;
using System.Linq;
using MissionPlanner.Utilities; // srtm クラスを使用するための using
using System.Threading.Tasks; // パラレル処理のための using
using System.Collections.Concurrent;

namespace MissionPlanner.Utilities
{
    public static class MissionEstimator
    {
        // サンプリング間隔（メートル）
        private const double SamplingInterval = 30.0;

        private static ConcurrentDictionary<(double lat, double lng), double> terrainCache = new ConcurrentDictionary<(double, double), double>();

        private static double GetCachedTerrainAltitude(double lat, double lng)
        {
            var key = (lat, lng);
            return terrainCache.GetOrAdd(key, k => srtm.getAltitude(k.lat, k.lng).alt);
        }

        // 3D距離を計算するメソッド
        public static double GetDistance3D(PointLatLngAlt p1, PointLatLngAlt p2)
        {
            double horizontalDistance = p1.GetDistance(p2);
            double altitudeDifference = p2.Alt - p1.Alt;

            return Math.Sqrt(horizontalDistance * horizontalDistance + altitudeDifference * altitudeDifference);
        }

        public static double GetDistance3D(Locationwp p1, Locationwp p2)
        {
            PointLatLngAlt point1 = new PointLatLngAlt(p1.lat, p1.lng, p1.alt);
            PointLatLngAlt point2 = new PointLatLngAlt(p2.lat, p2.lng, p2.alt);

            return GetDistance3D(point1, point2);
        }

        /// <summary>
        /// 指定されたポイントのフレームに基づいて絶対高度を取得します。
        /// </summary>
        /// <param name="homeAlt">ホームの絶対高度（ASL）</param>
        /// <param name="wp">ウェイポイント</param>
        /// <returns>絶対高度（ASL）</returns>
        private static double GetCorrectedAltitude(double homeAlt, Locationwp wp)
        {
            double correctedAlt = wp.alt;

            switch ((MAVLink.MAV_FRAME)wp.frame)
            {
                case MAVLink.MAV_FRAME.GLOBAL:
                case MAVLink.MAV_FRAME.GLOBAL_INT:
                    // GLOBALフレーム: ウェイポイントの高度をそのまま使用
                    correctedAlt = wp.alt;
                    break;

                case MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT:
                case MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT:
                    // RELATIVEフレーム: ホーム高度を加算してASLにする
                    correctedAlt += homeAlt;
                    break;

                case MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT:
                case MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT_INT:
                    // TERRAIN_ALTフレーム: 地形高度を取得して相対高度を加算
                    var terrainAlt = GetCachedTerrainAltitude(wp.lat, wp.lng);
                    correctedAlt += terrainAlt;
                    Console.WriteLine($"Terrain alt ({wp.lat}, {wp.lng}): {terrainAlt}");
                    break;

                default:
                    // サポートされていないフレームの場合のデフォルト動作
                    throw new NotSupportedException($"Frame type {wp.frame} is not supported for distance calculations.");
            }

            return correctedAlt;
        }

        /// <summary>
        /// 2つのウェイポイント間をサンプリングし、サンプルポイントのリストを返します。
        /// </summary>
        /// <param name="start">開始ポイント</param>
        /// <param name="end">終了ポイント</param>
        /// <returns>サンプルポイントのリスト</returns>
        public static List<PointLatLngAlt> SamplePath(PointLatLngAlt start, PointLatLngAlt end)
        {
            List<PointLatLngAlt> samples = new List<PointLatLngAlt>();

            double totalDistance = start.GetDistance(end);
            if (totalDistance == 0 || totalDistance > 30000 || (start.Lat == 0.0 && start.Lng == 0.0))
            {
                samples.Add(start);
                return samples;
            }

            int numSamples = (int)Math.Ceiling(totalDistance / SamplingInterval);
            double bearing = start.GetBearing(end);

            for (int i = 0; i <= numSamples; i++)
            {
                double distance = i * SamplingInterval;
                if (distance > totalDistance)
                    distance = totalDistance;

                // 緯度・経度を計算
                PointLatLngAlt sampledPoint = start.newpos(bearing, distance);

                // 高度を線形補間
                double altitude = start.Alt + (end.Alt - start.Alt) * (distance / totalDistance);
                sampledPoint.Alt = altitude;

                samples.Add(sampledPoint);
            }
            return samples;
        }

        /// <summary>
        /// 経路全体の3D距離と推定飛行時間を計算します。
        /// </summary>
        public static (double Total3DDistance, double totalEstimatedTimeSeconds) CalculateTotal3DDistanceWithTerrain(
            PointLatLngAlt home,
            List<Locationwp> wpCommandList,
            bool isTerrain,
            double rtlAltitude = 60.0,
            double horizontalSpeed = 10.0, // m/s
            double ascentSpeed = 2.5,      // m/s
            double descentSpeed = 2.5      // m/s
            )
        {
            if (!wpCommandList.Any())
            {
                return (0.0, 0.0);
            }

            double totalDistance = 0.0;
            double totalEstimatedTimeSeconds = 0.0;

            // 初期のpreviousCommandをホームポイントとして設定
            Locationwp previousCommand = new Locationwp
            {
                lat = home.Lat,
                lng = home.Lng,
                alt = (float)home.Alt, // home.Alt は float にキャスト
                frame = (int)MAVLink.MAV_FRAME.GLOBAL, // ホームポイントのフレームを設定
                id = (int)MAVLink.MAV_CMD.WAYPOINT // ホームポイントをWAYPOINTとして扱う
            };

            // 初期のpreviousWaypointもホームポイントと同じとする
            Locationwp previousWaypoint = previousCommand;

            foreach (var currentCommand in wpCommandList)
            {
                // 通常の距離計算を行う
                (double distance, double estimatedFlightTime) = CalculateDistanceAndFlightTimeForCommand(home, currentCommand, previousWaypoint, previousCommand, isTerrain, rtlAltitude, horizontalSpeed, ascentSpeed, descentSpeed);
                totalDistance += distance;
                totalEstimatedTimeSeconds += estimatedFlightTime;

                if (currentCommand.id == (int)MAVLink.MAV_CMD.TAKEOFF)
                {
                    // TAKEOFFコマンドの場合、previousCommandの高度のみを更新
                    previousCommand.alt += currentCommand.alt;
                    // 緯度・経度はホームポイントのまま
                }
                else
                {
                    // 通常のコマンドの場合、previousCommandを更新
                    previousCommand = currentCommand;
                }

                // 現在のコマンドがWAYPOINTまたはSPLINE_WAYPOINTの場合、previousWaypointを更新
                if (currentCommand.id == (int)MAVLink.MAV_CMD.WAYPOINT || currentCommand.id == (int)MAVLink.MAV_CMD.SPLINE_WAYPOINT)
                {
                    previousWaypoint = currentCommand;
                }
            }

            return (totalDistance, totalEstimatedTimeSeconds);
        }

        public static (double distance, double flightTime) CalculateDistanceAndFlightTimeForCommand(
            PointLatLngAlt home,
            Locationwp currentCommand,
            Locationwp previousWaypoint,
            Locationwp previousCommand,
            bool isTerrain,
            double rtlAltitude = 60.0,
            double horizontalSpeed = 10.0, // m/s
            double ascentSpeed = 2.5,      // m/s
            double descentSpeed = 2.5      // m/s
            )
        {
            double distance = 0.0;
            double flightTime = 0.0;

            switch (currentCommand.id)
            {
                case (int)MAVLink.MAV_CMD.WAYPOINT:
                case (int)MAVLink.MAV_CMD.SPLINE_WAYPOINT:
                    PointLatLngAlt currentPoint;

                    // 緯度、経度が0, 0の場合は前のWAYPOINTの座標を使用
                    if (currentCommand.lat == 0.0 && currentCommand.lng == 0.0)
                    {
                        // もし前のコマンドがTAKEOFFであれば、Homeの緯度・経度を使用
                        if (previousCommand.id == (int)MAVLink.MAV_CMD.TAKEOFF)
                        {
                            currentPoint = new PointLatLngAlt(home.Lat, home.Lng, GetCorrectedAltitude(home.Alt, currentCommand), "GLOBAL");
                        }
                        else
                        {
                            currentPoint = new PointLatLngAlt(previousWaypoint.lat, previousWaypoint.lng, GetCorrectedAltitude(home.Alt, currentCommand), "GLOBAL");
                        }
                    }
                    else
                    {
                        // フレームに基づいて高度を補正
                        double correctedAlt = GetCorrectedAltitude(home.Alt, currentCommand);
                        string frameName = ((MAVLink.MAV_FRAME)currentCommand.frame).ToString();
                        currentPoint = new PointLatLngAlt(currentCommand.lat, currentCommand.lng, correctedAlt, frameName);
                    }


                    if (((MAVLink.MAV_FRAME)currentCommand.frame) == MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT ||
                             ((MAVLink.MAV_FRAME)currentCommand.frame) == MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT_INT)
                    {
                        var previousCorrectedAltitude = previousCommand.alt;
                        if (((MAVLink.MAV_FRAME)previousCommand.frame) == MAVLink.MAV_FRAME.GLOBAL ||
                             ((MAVLink.MAV_FRAME)previousCommand.frame) == MAVLink.MAV_FRAME.GLOBAL_INT)
                        {
                            previousCorrectedAltitude -= (float)GetCachedTerrainAltitude(previousCommand.lat, previousCommand.lng);
                        }
                        PointLatLngAlt previousPoint = new PointLatLngAlt(previousCommand.lat, previousCommand.lng, previousCorrectedAltitude);
                        currentPoint.Alt = currentCommand.alt;
                        currentPoint.Tag = ((MAVLink.MAV_FRAME)currentCommand.frame).ToString();

                        // 経路をサンプリング
                        List<PointLatLngAlt> sampledPoints = SamplePath(previousPoint, currentPoint);

                        if (sampledPoints.Count >= 2)
                        {
                            PointLatLngAlt previousSample = sampledPoints.First();
                            previousSample.Alt += GetCachedTerrainAltitude(previousSample.Lat, previousSample.Lng);
                            for (int i = 1; i < sampledPoints.Count; i++)
                            {
                                var currentSample = sampledPoints[i];
                                currentSample.Alt += GetCachedTerrainAltitude(currentSample.Lat, currentSample.Lng);

                                // 前のサンプルポイントとの3D距離を計算
                                distance += GetDistance3D(previousSample, currentSample);

                                double horizontalSampleDistance = currentSample.GetDistance(previousSample);
                                double verticalSampleDistance = currentSample.Alt - previousSample.Alt;
                                double segmentTime = CalculateSegmentFlightTime(horizontalSampleDistance, verticalSampleDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                                flightTime += segmentTime;

                                Console.WriteLine($"Sampled Point1 ({previousSample.Lat}, {previousSample.Lng}, {previousSample.Alt}) " +
                                                  $"Point2 ({currentSample.Lat}, {currentSample.Lng}, {currentSample.Alt}) Distance: {distance}");
                                previousSample = currentSample;
                            }
                        }
                        else
                        {
                            Console.WriteLine("Sampled points insufficient for distance calculation.");
                        }
                    }
                    else
                    {
                        // 前のコマンドのフレーム名を取得
                        double correctedPreviousAltitude = GetCorrectedAltitude(home.Alt, previousCommand);
                        string previousFrameName = ((MAVLink.MAV_FRAME)previousCommand.frame).ToString();
                        PointLatLngAlt previousPoint = new PointLatLngAlt(previousCommand.lat, previousCommand.lng, correctedPreviousAltitude, previousFrameName);

                        distance = GetDistance3D(currentPoint, previousPoint);

                        double horizontalDistance = currentPoint.GetDistance(previousPoint);
                        double verticalSegmentDistance = currentPoint.Alt - previousPoint.Alt;
                        flightTime = CalculateSegmentFlightTime(horizontalDistance, verticalSegmentDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                    }
                    break;

                case (int)MAVLink.MAV_CMD.TAKEOFF:
                    // 高度補正を行ってから距離を計算
                    double correctedCurrentAlt = GetCorrectedAltitude(home.Alt, currentCommand);
                    double correctedPreviousAlt = GetCorrectedAltitude(home.Alt, previousCommand);

                    distance = Math.Abs(correctedCurrentAlt - correctedPreviousAlt);
                    double verticalDistance = correctedCurrentAlt - correctedPreviousAlt;
                    flightTime = CalculateSegmentFlightTime(0, verticalDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                    break;

                case (int)MAVLink.MAV_CMD.LAND:
                    // LANDコマンド: 現在位置から地面への垂直降下距離を計算
                    double landAltitude = GetCorrectedAltitude(home.Alt, currentCommand);
                    correctedPreviousAlt = GetCorrectedAltitude(home.Alt, previousCommand);
                    distance = Math.Abs(correctedPreviousAlt - landAltitude);
                    verticalDistance = landAltitude - correctedPreviousAlt;
                    flightTime = CalculateSegmentFlightTime(0, verticalDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                    break;

                case (int)MAVLink.MAV_CMD.RETURN_TO_LAUNCH:
                    // RETURN_TO_LAUNCHコマンドの処理
                    (distance, flightTime) = CalculateRTLDistance(home, previousCommand, currentCommand, isTerrain, rtlAltitude, horizontalSpeed, ascentSpeed, descentSpeed);
                    break;

                // 他のコマンドIDに対する処理を追加
                case (int)MAVLink.MAV_CMD.PAYLOAD_PLACE:
                case (int)MAVLink.MAV_CMD.DO_JUMP:
                    // 他の未対応コマンドもここに追加可能
                    distance = 0.0;
                    break;

                default:
                    // 距離に影響しないコマンドは無視
                    distance = 0.0;
                    flightTime = 0.0;
                    break;
            }

            return (distance, flightTime);
        }

        private static double CalculateSegmentFlightTime(double horizontalDistance, double verticalDistance, double horizontalSpeed, double ascentSpeed, double descentSpeed)
        {
            double horizontalTime = horizontalDistance / horizontalSpeed;
            double verticalTime = verticalDistance > 0
                ? verticalDistance / ascentSpeed
                : -verticalDistance / descentSpeed;
            return Math.Max(horizontalTime, verticalTime);
        }

        /// <summary>
        /// RETURN_TO_LAUNCHコマンドの総距離と飛行時間を計算します。
        /// </summary>
        private static (double distance, double flightTime) CalculateRTLDistance(
            PointLatLngAlt home,
            Locationwp previousCommand,
            Locationwp currentCommand,
            bool isTerrain,
            double rtlAltitude = 60.0,
            double horizontalSpeed = 10.0, // m/s
            double ascentSpeed = 2.5,      // m/s
            double descentSpeed = 2.5      // m/s
            )
        {
            double totalRTLDistance = 0.0;
            double totalFlightTime = 0.0;

            // 高度を補正
            double correctedPreviousAlt = GetCorrectedAltitude(home.Alt, previousCommand);
            var correctedRtlAltitude = rtlAltitude + (isTerrain ? GetCachedTerrainAltitude(previousCommand.lat, previousCommand.lng) : home.Alt);

            // 1. 上昇
            double ascentDistance = 0.0;
            if (correctedPreviousAlt < correctedRtlAltitude)
            {
                ascentDistance = correctedRtlAltitude - correctedPreviousAlt;
                totalRTLDistance += ascentDistance;
                totalFlightTime += ascentDistance / ascentSpeed;
                Console.WriteLine($"Ascent Distance: {ascentDistance} meters to reach desired altitude of {correctedRtlAltitude} meters.");
            }

            // 2. ホームポイントへの水平移動
            double currentAltitudeForMove = correctedPreviousAlt < correctedRtlAltitude ? correctedRtlAltitude : correctedPreviousAlt;
            PointLatLngAlt currentAtDesiredAlt = new PointLatLngAlt(
                previousCommand.lat,
                previousCommand.lng,
                isTerrain ? rtlAltitude : currentAltitudeForMove,
                isTerrain ? "GLOBAL_TERRAIN_ALT" : "GLOBAL"
            );

            PointLatLngAlt homeAtDesiredAlt = new PointLatLngAlt(
                home.Lat,
                home.Lng,
                isTerrain ? rtlAltitude : correctedRtlAltitude,
                isTerrain ? "GLOBAL_TERRAIN_ALT" : "GLOBAL"
            );

            if (isTerrain)
            {
                // 地形を考慮してサンプリングパスを使用
                List<PointLatLngAlt> sampledPoints = SamplePath(currentAtDesiredAlt, homeAtDesiredAlt);

                if (sampledPoints.Count < 2)
                {
                    Console.WriteLine("Sampled points insufficient for distance calculation.");
                }
                else
                {
                    PointLatLngAlt previousPoint = sampledPoints.First();
                    previousPoint.Alt += GetCachedTerrainAltitude(previousPoint.Lat, previousPoint.Lng);

                    for (int i = 1; i < sampledPoints.Count; i++)
                    {
                        var sampledPoint = sampledPoints[i];
                        sampledPoint.Alt += GetCachedTerrainAltitude(sampledPoint.Lat, sampledPoint.Lng);

                        // サンプルポイント間の3D距離を計算
                        double distance = GetDistance3D(previousPoint, sampledPoint);
                        totalRTLDistance += distance;

                        double horizontalDistance = sampledPoint.GetDistance(previousPoint);
                        double verticalDistance = sampledPoint.Alt - previousPoint.Alt;
                        double segmentTime = CalculateSegmentFlightTime(horizontalDistance, verticalDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                        totalFlightTime += segmentTime;

                        Console.WriteLine($"Sampled Point1 ({previousPoint.Lat}, {previousPoint.Lng}, {previousPoint.Alt}) " +
                                          $"Point2 ({sampledPoint.Lat}, {sampledPoint.Lng}, {sampledPoint.Alt}) Distance: {distance}");

                        previousPoint = sampledPoint;
                    }
                }
            }
            else
            {
                // 通常の直線距離を計算
                double distance = GetDistance3D(currentAtDesiredAlt, homeAtDesiredAlt);
                totalRTLDistance += distance;

                double horizontalDistance = currentAtDesiredAlt.GetDistance(homeAtDesiredAlt);
                double verticalDistance = homeAtDesiredAlt.Alt - currentAtDesiredAlt.Alt;

                double segmentTime = CalculateSegmentFlightTime(horizontalDistance, verticalDistance, horizontalSpeed, ascentSpeed, descentSpeed);
                totalFlightTime += segmentTime;

                Console.WriteLine($"Straight Line Distance: {distance} meters.");
            }

            // 3. 降下
            double descentDistance = rtlAltitude; // ホームポイントは地面にあると仮定
            totalRTLDistance += descentDistance;
            totalFlightTime += descentDistance / descentSpeed;
            Console.WriteLine($"Descent Distance: {descentDistance} meters to land.");

            return (totalRTLDistance, totalFlightTime);
        }
    }
}