using System;
using static MAVLink;
using System.Collections.Generic;
using System.Linq;

namespace MissionPlanner.Utilities
{
    public static class MissionEstimator
    {
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

        public static double CalculateTotal3DDistance(PointLatLngAlt home, List<Locationwp> wpCommandList)
        {
            if (!wpCommandList.Any())
            {
                return 0.0;
            }

            double totalDistance = 0.0;

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
                double distance = CalculateDistanceForCommand(home, currentCommand, previousWaypoint, previousCommand);
                totalDistance += distance;

                if (currentCommand.id == (int)MAVLink.MAV_CMD.TAKEOFF)
                {
                    // TAKEOFFコマンドの場合、previousCommandの高度のみを更新
                    previousCommand.alt = currentCommand.alt;
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

            return totalDistance;
        }

        // 特定のミッションコマンドに対する距離を計算するメソッド
        public static double CalculateDistanceForCommand(PointLatLngAlt home, Locationwp currentCommand, Locationwp previousWaypoint, Locationwp previousCommand)
        {
            double distance = 0.0;

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
                            currentPoint = new PointLatLngAlt(home.Lat, home.Lng, currentCommand.alt, "GLOBAL");
                        }
                        else
                        {
                            currentPoint = new PointLatLngAlt(previousWaypoint.lat, previousWaypoint.lng, currentCommand.alt, "GLOBAL");
                        }
                    }
                    else
                    {
                        // MAV_FRAME列挙型を使用してフレーム名を取得
                        string frameName = ((MAVLink.MAV_FRAME)currentCommand.frame).ToString();
                        currentPoint = new PointLatLngAlt(currentCommand.lat, currentCommand.lng, currentCommand.alt, frameName);
                    }

                    // 前のコマンドのフレーム名を取得
                    string previousFrameName = ((MAVLink.MAV_FRAME)previousCommand.frame).ToString();
                    PointLatLngAlt previousPoint = new PointLatLngAlt(previousCommand.lat, previousCommand.lng, previousCommand.alt, previousFrameName);
                    distance = GetDistance3D(currentPoint, previousPoint);
                    break;

                case (int)MAVLink.MAV_CMD.TAKEOFF:
                    // 高度補正を行ってから距離を計算
                    double correctedCurrentAlt = GetCorrectedAltitude(home.Alt, currentCommand);
                    double correctedPreviousAlt = GetCorrectedAltitude(home.Alt, previousCommand);

                    distance = Math.Abs(correctedCurrentAlt - correctedPreviousAlt);
                    break;

                case (int)MAVLink.MAV_CMD.LAND:
                    // LANDコマンド: 現在位置から地面への垂直降下距離を計算
                    // 仮定: LANDコマンドのcurrentCommand.altは0とする
                    distance = Math.Abs(currentCommand.alt - previousCommand.alt);
                    break;

                case (int)MAVLink.MAV_CMD.RETURN_TO_LAUNCH:
                    // RETURN_TO_LAUNCHコマンドの処理
                    distance = CalculateRTLDistance(home, previousCommand, currentCommand);
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
                    break;
            }

            return distance;
        }

        /// <summary>
        /// RETURN_TO_LAUNCHコマンドの総距離を計算します。
        /// </summary>
        /// <param name="home">ホームポイント</param>
        /// <param name="previousCommand">前のコマンド</param>
        /// <param name="currentCommand">現在のRTLコマンド</param>
        /// <returns>RTLコマンドに対応する総距離（メートル）</returns>
        private static double CalculateRTLDistance(PointLatLngAlt home, Locationwp previousCommand, Locationwp currentCommand)
        {
            double totalRTLDistance = 0.0;
            double desiredAltitude = 60.0; // デフォルトのRTL上昇高度（メートル）

            // 1. 上昇: 現在の高度が60m未満の場合、60mまで上昇
            double ascentDistance = 0.0;
            if (previousCommand.alt < desiredAltitude)
            {
                ascentDistance = desiredAltitude - previousCommand.alt;
                totalRTLDistance += ascentDistance;
            }

            // 2. ホームポイントへの水平移動（高度は60m）
            double currentAltitudeForMove = previousCommand.alt < desiredAltitude ? desiredAltitude : previousCommand.alt;
            PointLatLngAlt currentAtDesiredAlt = new PointLatLngAlt(
                previousCommand.lat,
                previousCommand.lng,
                currentAltitudeForMove,
                "GLOBAL" // フレームを固定値に設定
            );

            PointLatLngAlt homeAtDesiredAlt = new PointLatLngAlt(
                home.Lat,
                home.Lng,
                desiredAltitude,
                "GLOBAL" // フレームを固定値に設定
            );

            double horizontalDistance = GetDistance3D(currentAtDesiredAlt, homeAtDesiredAlt);
            totalRTLDistance += horizontalDistance;

            // 3. 降下: ホームポイントから地面へ降下
            double descentDistance = desiredAltitude; // ホームポイントは地面にあると仮定
            totalRTLDistance += descentDistance;

            return totalRTLDistance;
        }

        // 高度補正関数
        private static double GetCorrectedAltitude(double homeAlt, Locationwp wp)
        {
            double correctedAlt = wp.alt;

            switch ((MAVLink.MAV_FRAME)wp.frame)
            {
                case MAVLink.MAV_FRAME.GLOBAL:
                case MAVLink.MAV_FRAME.GLOBAL_INT:
                case MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT:
                case MAVLink.MAV_FRAME.GLOBAL_TERRAIN_ALT_INT:
                    // GLOBALまたはTERRAIN ALTフレーム: ウェイポイントの高度をそのまま使用
                    correctedAlt = wp.alt;
                    break;

                case MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT:
                case MAVLink.MAV_FRAME.GLOBAL_RELATIVE_ALT_INT:
                    // RELATIVEフレーム: ホーム高度を加算してASLにする
                    correctedAlt += homeAlt;
                    break;

                default:
                    // サポートされていないフレームの場合のデフォルト動作
                    throw new NotSupportedException($"Frame type {wp.frame} is not supported for distance calculations.");
            }

            return correctedAlt;
        }
    }
}
