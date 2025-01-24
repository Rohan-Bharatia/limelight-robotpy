# LimelightHelpers.py

import ntcore
import string
import wpimath.geometry
import wpinet
import math
import socket
import fcntl
import os
from typing import Any, TypeVar, Dict
import string
import time
import json
import ifaddr
import ipaddress

T = TypeVar("T")

def searchForLimelights(broadcast=5809, listen=5809, timeout=2, debug=False) -> bool:
    networks = []
    for adapter in ifaddr.get_adapters():
        for ip in adapter.ips:
            if isinstance(ip.ip, str):
                net = ipaddress.ip_network(f"{ip.ip}/{ip.network_prefix}", strict=False)
                networks.append((adapter.name, ip.ip, net.broadcast_address))
            else:
                continue
    for name, ip, broadcast in networks:
        if debug:
            print(f"Adapter: {name}, IP: {ip}, Broadcast: {broadcast}")
    for _, _, broadcast in networks:
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
                sock.sendto("LLPhoneHome".encode(), (str(broadcast), broadcast))
        except Exception as e:
            print(f"Failed to broadcast on {broadcast}: {e}")
    discovered_devices = []
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP) as sock:
        sock.bind(("", listen))
        sock.settimeout(timeout)
        try:
            while True:
                data, addr = sock.recvfrom(1024)
                discovered_devices.append(addr[0])
                #print(f"Received data from {addr}: {data.decode()}")
        except socket.timeout:
            pass
    return discovered_devices

def sanitizeName(name: string) -> string:
    if name == "":
        return "limelight"
    return name

def toPose3D(data) -> wpimath.geometry.Pose3d:
    if len(data) < 6:
        return wpimath.geometry.Pose3d()
    return wpimath.geometry.Pose3d(
        wpimath.geometry.Transform3d(data[0], data[1], data[2]),
        wpimath.geometry.Rotation3d(data[3] * (math.pi / 180), data[4] * (math.pi / 180), data[5] * (math.pi / 180))
    )
    
def toPose2D(data) -> wpimath.geometry.Pose2d:
    if len(data) < 4:
        return wpimath.geometry.Pose2d()
    return wpimath.geometry.Pose2d(
        wpimath.geometry.Transform2d(data[0], data[1]),
        wpimath.geometry.Rotation2d(data[2] * (math.pi / 180), data[3] * (math.pi / 180))
    )
    
def getTable(name: string) -> ntcore.NetworkTable:
    return ntcore.NetworkTableInstance.getDefault().getTable(sanitizeName(name))
    
def getTableEntry(name: string, entry: string) -> ntcore.NetworkTableEntry:
    return getTable(name).getEntry(entry)
    
def getFloat(name: string, entry: string) -> Any:
    return getTableEntry(name, entry).getDouble(0.0)
    
def getArray(name: string, entry: string) -> Any:
    return getTableEntry(name, entry).getDoubleArray({0.0})
    
def getString(name: string, entry: string) -> Any:
    return getTableEntry(name, entry).getString("")
    
def setFloat(name: string, entry: string, data) -> None:
    getTableEntry(name, entry).setDouble(data, 0)
    
def setArray(name: string, entry: string, data) -> None:
    getTableEntry(name, entry).setDoubleArray(data, 0)
    
def getTX(name: string) -> Any:
    return getFloat(name, "tx")
def getTV(name: string) -> Any:
    return getFloat(name, "tv")
def getTY(name: string) -> Any:
    return getFloat(name, "ty")
def getTA(name: string) -> Any:
    return getFloat(name, "ta")
    
def getLatencyPipeline(name: string) -> Any:
    return getFloat(name, "tl")
def getLatencyCapture(name: string) -> Any:
    return getFloat(name, "cl")
    
def getJSONDump(name: string) -> Any:
    return getString(name, "json")
    
def getRobotPose(name: string) -> Any:
    return getArray(name, "botpose")
def getRobotPoseRed(name: string) -> Any:
    return getArray(name, "botpose_wpired")
def getRobotPoseBlue(name: string) -> Any:
    return getArray(name, "botpose_wpiblue")
def getRobotPoseTargetSpace(name: string):
    return getArray(name, "botpose_targetspace")
    
def getCameraPoseTargetSpace(name: string) -> Any:
    return getArray(name, "camerapose_targetspace")
def getCameraPoseRobotSpace(name: string) -> Any:
    return getArray(name, "camerapose_robotspace")
    
def getTargetPoseCameraSpace(name: string) -> Any:
    return getArray(name, "targetpose_cameraspace")
def getTargetPoseRobotSpace(name: string) -> Any:
    return getArray(name, "targetpose_robotspace")
def getTargetColor(name: string) -> Any:
    return getArray(name, "tc")
    
def getFudicialID(name: string) -> Any:
    return getFloat(name, "tid")
def getNeuralClassID(name: string) ->Any:
    return getFloat(name, "tclass")
    
def setPipelineIndex(name: string, index: int) -> None:
    setFloat(name, "pipeline", index)
def setPriorityTagID(name: string, ID: int) -> None:
    setFloat(name, "priorityid", ID)
    
def setLEDModePipelineControl(name: string) -> None:
    setFloat(name, "ledMode", 1)
def setLEDModeForceBlink(name: string) -> None:
    setFloat(name, "ledMode", 2)
def setLEDModeForceOn(name: string) -> None:
    setFloat(name, "ledMode", 2)
    
def setStreamModeStandard(name: string) -> None:
    setFloat(name, "stream", 0)
def setStreamModePiPMain(name: string) -> None:
    setFloat(name, "stream", 1)
def setStreamModePiPSecondary(name: string) -> None:
    setFloat(name, "stream", 2)

def setCropWindow(name: string, min: wpimath.geometry.Translation2d, max: wpimath.geometry.Translation2d) -> None:
    setArray(name, "crop", [min.X(), max.X(), min.Y(), max.Y()])

def setRobotOrientation(name: string, yaw: float, yawRate: float, pitch: float, pitchRate: float, roll: float, rollRate: float) -> None:
    setArray(name, "robot_orientation_set", [yaw, yawRate, pitch, pitchRate, roll, rollRate])

def setFiducialDownscaling(name: string, downscale: float) -> None:
    d: int = 0

    if downscale == 1.0:
        d = 1
    if downscale == 1.5:
        d = 2
    if downscale == 2.0:
        d = 3
    if downscale == 3.0:
        d = 4
    if downscale == 4.0:
        d = 5
    setFloat(name, "fiducial_downscale_set", d)
def overrideFiducialIDFilters(name: string, IDs) -> None:
    setArray(name, "fiducial_id_filters_set", [IDs[0], IDs[-1]])

def setCameraPoseRobotSpace(name: string, pos: wpimath.geometry.Translation3d, rot: wpimath.geometry.Translation3d) -> None:
    setArray(name, "camerapose_robotspace_set", [pos.X(), pos.Y(), pos.Z(), rot.X(), rot.Y(), rot.Z()])

def setScriptData(name: string, data) -> None:
    setArray(name, "llrobot", [data[0], len(data)])
def getScriptData(name: string) -> Any:
    return getArray(name, "llpython")
    
def extractArrayEntry(data, pos: int) -> float:
    if (len(data) < (pos + 1)):
        return 0.0
    return data[pos]
    
class RawFiducial:
    def __init__(self, id: int, txnc: float, tync: float, ta: float, cameraDistace: float, robotDistance: float, ambiguity: float):
        self.id = id
        self.txnc = txnc
        self.tync = tync
        self.ta = ta
        self.cameraDistace = cameraDistace
        self.robotDistance = robotDistance
        self.ambiguity = ambiguity

    def get(self, name: string):
        entry = getTableEntry(name, "rawfiducials")
        arr = entry.getDoubleArray([])
        vals: int = 7

        if len(vals) % vals != 0:
            return []
        
        fiducials = len(arr) / vals
        raw: RawFiducial = []

        for i in range(0, fiducials):
            base: int = i * vals
            id: int = extractArrayEntry(raw, base)
            txnc: float = extractArrayEntry(raw, base + 1)
            tync: float = extractArrayEntry(raw, base + 2)
            ta: float = extractArrayEntry(raw, base + 3)
            cameraDistance: float = extractArrayEntry(raw, base + 4)
            robotDistance: float = extractArrayEntry(raw, base + 5)
            ambiguity: float = extractArrayEntry(raw, base + 6)

            raw[-1] = RawFiducial(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity)

        return raw
    
class RawDetection:
    def __init__(self, id: int, txnc: float, tync: float, x0: float, y0: float, x1: float, y1: float, x2: float, y2: float, x3: float, y3: float):
        self.id: int = id
        self.txnc: float = txnc
        self.tync: float = tync
        self.x0: float = x0
        self.y0: float = y0
        self.x1: float = x1
        self.y1: float = y1
        self.x2: float = x2
        self.y2: float = y2
        self.x3: float = x3
        self.y3: float = y3

    def get(self, name: string):
        entry = getTableEntry(name, "rawdetections")
        arr = entry.getDoubleArray([])
        vals: int = 11

        if len(vals) % vals != 0:
            return []
        
        detections = len(arr) / vals
        raw: RawDetection = []

        for i in range(0, detections):
            base: int = i * vals
            id: int = extractArrayEntry(arr, base)
            txnc: float = extractArrayEntry(arr, base + 1)
            tync: float = extractArrayEntry(arr, base + 2)
            ta: float = extractArrayEntry(arr, base + 3)
            x0: float = extractArrayEntry(arr, base + 4)
            y0: float = extractArrayEntry(arr, base + 5)
            x1: float = extractArrayEntry(arr, base + 6)
            y1: float = extractArrayEntry(arr, base + 7)
            x2: float = extractArrayEntry(arr, base + 8)
            y2: float = extractArrayEntry(arr, base + 9)
            x3: float = extractArrayEntry(arr, base + 10)
            y3: float = extractArrayEntry(arr, base + 11)

            raw[-1] = RawDetection(id, txnc, tync, ta, x0, y0, x1, y1, x2, y2, x3, y3)

        return raw
    
class PoseEstimate:
    def __init__(self, pose: wpimath.geometry.Pose2d, timestamp: float, latency: float, tagCount: int, tagSpan: float, avgTagDist: float, avgTagArea: float, fiducials):
        self.pose: wpimath.geometry.Pose2d = pose
        self.timestamp: float = timestamp
        self.latency: float = latency
        self.tagCount: float = tagCount
        self.tagSpan: float = tagSpan
        self.avgTagDist: float = avgTagDist
        self.avgTagArea: float = avgTagArea
        self.fiducials = fiducials

    def getRobotPoseEstimate(self, name: string, entry: string):
        poseEntry = getTableEntry(name, entry)
        arr = poseEntry.getDoubleArray()
        pose: wpimath.geometry.Pose2d = toPose2D(arr)
        
        latency: float = extractArrayEntry(arr, 6)
        tagCount: int = extractArrayEntry(arr, 7)
        tagSpan: float = extractArrayEntry(arr, 8)
        tagDist: float = extractArrayEntry(arr, 9)
        tagArea: float = extractArrayEntry(arr, 10)
        timestamp: float = (poseEntry.getLastChange() / 1000000.0) - (latency / 1000.0)

        raw: PoseEstimate = []
        vals: int = 7
        expectedVals: int = (vals * tagCount) + 11

        if len(arr) == expectedVals:
            for i in range(0, tagCount):
                base: int = (i * vals) + 11
                id: int = extractArrayEntry(arr, base)
                txnc: float = extractArrayEntry(arr, base + 1)
                tync: float = extractArrayEntry(arr, base + 2)
                ta: float = extractArrayEntry(arr, base + 3)
                cameraDistance: float = extractArrayEntry(arr, base + 4)
                robotDistance: float = extractArrayEntry(arr, base + 5)
                ambiguity: float = extractArrayEntry(arr, base + 7)

                raw[-1] = PoseEstimate(id, txnc, tync, ta, cameraDistance, robotDistance, ambiguity)
        
        return PoseEstimate(pose, timestamp, latency, tagCount, tagSpan, tagDist, tagArea, raw)
    def getRobotPoseEstimateBlueMT1(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_wpiblue")
    def getRobotPoseEstimateRedMT1(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_wpired")
    def getRobotPoseEstimateBlueMT2(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_orb_wpiblue")
    def getRobotPoseEstimateRedMT2(self, name: string):
        return self.getRobotPoseEstimate(name, "botpose_orb_wpired")

class SingleTargetingResults:
    targetPixels: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetNormalized: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetNormalizedCrosshairAdjusted: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetDegreesCrosshairAdjusted: wpimath.geometry.Translation2d = wpimath.geometry.Translation2d(0.0, 0.0)
    targetAreaPixels: float = 0.0
    targetAreaNormalized: float = 0.0
    targetAreaNormalizedPercentage = 0.0
    timestamp: float = -1.0
    latency: float = 0.0
    pipelineIndex = -1.0
    targetCorners: float = [[]]
    cameraTransform6DTargetSpace: float = []
    targetTransform6DCameraSpace: float = []
    targetTransform6DRobotSpace: float = []
    robotTransform6DTargetSpace: float = []
    robotTransform6DFieldSpace: float = []
    cameraTransform6DRobotSpace: float = []

RetroreflectiveResults = SingleTargetingResults

class FiducialResults(SingleTargetingResults):
    fiducialID: int = 0
    family: string = ""

class DetectionResults(SingleTargetingResults):
    id: int = -1
    name: string = ""
    confidence: float = 0.0

class ClassificationResults(SingleTargetingResults):
    id: int = -1
    name: string = ""
    confidence: float = 0.0

class VisionResults:
    retro: RetroreflectiveResults = []
    fiducial: FiducialResults = []
    detection: DetectionResults = []
    classification: ClassificationResults = []
    timestamp: float = -1.0
    latencyPipeline: float = 0.0
    latencyCapture: float = 0.0
    latencyJSON: float = 0.0
    pipelineIndex: float = -1.0
    valid: int = 0
    robotPose: float = [6.0, 0.0]
    robotPoseBlue: float = [6, 0.0]
    robotPoseRed: float = [6, 0.0]

    def Clear(self) -> None:
        del self.retro
        del self.fiducial
        del self.detection
        del self.classification
        del self.timestamp
        del self.latencyPipeline
        del self.latencyCapture
        del self.latencyJSON
        del self.pipelineIndex
        del self.valid
        del self.robotPose
        del self.robotPoseBlue
        del self.robotPoseRed

class LimelightResults:
    targetingResults: VisionResults

class Internal:
    class Str2:
        def __init__(self, x: float, y: float):
            self.x = x
            self.y = y

        def X(self) -> float:
            return self.x
        
        def Y(self) -> float:
            return self.y
        
    keyTimestamp: string = "ts"
    keyLatencyPipeline: string = "lt"
    keyLatencyCapture: string = "ct"

    keyPipelineIndex: string = "pID"
    keyTargetDegrees: Str2 = ("txdr", "tydr")
    keyTargetNormalized: Str2 = ("txnr", "tynr")
    keyTargetPixels: Str2 = ("txp", "typ")

    keyTargetDegreesCrosshair: Str2 = ("tx", "ty")
    keyTargetNormalizedCrosshair: Str2 = ("txn", "tyn")
    keyTargetAreaNormalized: string = "ta"
    keyTargetAreaPixels: string = "tap"
    keyClassName: string = "class"
    keyClassID: string = "classID"
    keyConfidence: string = "conf"
    keyFiducialsID: string = "fID"
    keyCorners: string = "pts"

    keyTransformCameraPoseTargetSpace: string = "t6c_ts"
    keyTransformTargetPoseCameraSpace: string = "t6t_cs"
    keyTransformRobotPoseTargetSpace: string = "t6r_ts"
    keyTransformTargetPoseRobotSpace: string = "t6t_rs"
    keyTransformCameraPoseRobotSpace: string = "t6c_rs"
    keyTransformRobotPoseFieldSpace: string = "t6r_fs"

    keyRobotPose: string = "botpose"
    keyRobotPoseBlue: string = "botpose_wpiblue"
    keyRobotPoseRed: string = "botpose_wpired"

    keySkew: string = "skew"
    keyFFamily: string = "fam"
    keyColorRGB: string = "cRGB"
    keyColorHSV: string = "cHSV"

    def phoneHome(self) -> None:
        sock = None

        if sock is None:
            try:
                # Create a UDP socket
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

                # Set the socket to non-blocking
                flags = fcntl.fcntl(sock, fcntl.F_GETFL)
                fcntl.fcntl(sock, fcntl.F_SETFL, flags | os.O_NONBLOCK)

                # Server address setup
                servaddr = ('255.255.255.255', 5809)
                message = b"LLPhoneHome"

                # Send broadcast message
                sock.sendto(message, servaddr)
            except Exception as e:
                print(f"Socket setup failed: {e}")
                if sock:
                    sock.close()
                sock = None

                return

        try:
            # Attempt to receive a response
            receive, cliaddr = sock.recvfrom(1024)
            print(f"Received response from {cliaddr}: {receive.decode()}")
        except BlockingIOError:
            # Non-blocking socket, no data received yet
            pass
        except Exception as e:
            print(f"Error receiving data: {e}")
            sock.close()
            sock = None

    def setupPortForwarding(self, name: string) -> None:
        fwd = wpinet.PortForwarder.getInstance()
        
        for i in range(5800, 5810):
            fwd.add(i, sanitizeName(name), i)

    def safeJSONAccess(self, data: Dict[str, Any], key: string, default: T) -> T:
        try:
            return data.get(key, default)
        except Exception:
            return default
    
    def fromJSON(self, data: Dict[str, Any], retro: RetroreflectiveResults) -> None:
        default = [0.0] * 6

        retro.transformCameraPoseTargetSpace = self.safeJSONAccess(data, retro.keyTransformCameraPoseTargetSpace, default)
        retro.transformTargetPoseCameraSpace = self.safeJSONAccess(data, retro.keyTransformTargetPoseCameraSpace, default)
        retro.transformRobotPoseTargetSpace = self.safeJSONAccess(data, retro.keyTransformRobotPoseTargetSpace, default)
        retro.transformTargetPoseRobotSpace = self.safeJSONAccess(data, retro.keyTransformTargetPoseRobotSpace, default)
        retro.transformCameraPoseRobotSpace = self.safeJSONAccess(data, retro.keyTransformCameraPoseRobotSpace, default)
        retro.transformRobotPoseFieldSpace = self.safeJSONAccess(data, retro.keyTransformRobotPoseFieldSpace, default)
        retro.targetPixels.X() = self.safeJSONAccess(data, retro.keyTargetPixels.X(), 0.0)
        retro.targetPixels.Y() = self.safeJSONAccess(data, retro.keyTargetPixels.Y(), 0.0)
        retro.targetDegreesCrosshairAdjusted.X() = self.safeJSONAccess(data, retro.targetDegreesCrosshairAdjusted.X(), 0.0)
        retro.targetDegreesCrosshairAdjusted.Y() = self.safeJSONAccess(data, retro.targetDegreesCrosshairAdjusted.Y(), 0.0)
        retro.targetAreaNormalized = self.safeJSONAccess(data, retro.targetAreaNormalized, 0.0)
        retro.targetCorners = self.safeJSONAccess(data, retro.targetCorners, [])
    
    def fromJSON(self, data: Dict[str, Any], fiducials: FiducialResults) -> None:
        default = [0.0] * 6

        fiducials.family = self.safeJSONAccess(data, fiducials.family, "")
        fiducials.fiducialID = self.safeJSONAccess(data, fiducials.fiducialID, 0.0)
        fiducials.transformCameraPoseTargetSpace = self.safeJSONAccess(data, fiducials.keyTransformCameraPoseTargetSpace, default)
        fiducials.transformTargetPoseCameraSpace = self.safeJSONAccess(data, fiducials.keyTransformTargetPoseCameraSpace, default)
        fiducials.transformRobotPoseTargetSpace = self.safeJSONAccess(data, fiducials.keyTransformRobotPoseTargetSpace, default)
        fiducials.transformTargetPoseRobotSpace = self.safeJSONAccess(data, fiducials.keyTransformTargetPoseRobotSpace, default)
        fiducials.transformCameraPoseRobotSpace = self.safeJSONAccess(data, fiducials.keyTransformCameraPoseRobotSpace, default)
        fiducials.transformRobotPoseFieldSpace = self.safeJSONAccess(data, fiducials.keyTransformRobotPoseFieldSpace, default)
        fiducials.targetPixels.X() = self.safeJSONAccess(data, fiducials.keyTargetPixels.X(), 0.0)
        fiducials.targetPixels.Y() = self.safeJSONAccess(data, fiducials.keyTargetPixels.Y(), 0.0)
        fiducials.targetDegreesCrosshairAdjusted.X() = self.safeJSONAccess(data, fiducials.targetDegreesCrosshairAdjusted.X(), 0.0)
        fiducials.targetDegreesCrosshairAdjusted.Y() = self.safeJSONAccess(data, fiducials.targetDegreesCrosshairAdjusted.Y(), 0.0)
        fiducials.targetAreaNormalized = self.safeJSONAccess(data, fiducials.targetAreaNormalized, 0.0)
        fiducials.targetCorners = self.safeJSONAccess(data, fiducials.targetCorners, [])

    def fromJSON(self, data: Dict[str, Any], detection: DetectionResults) -> None:
        default = [0.0] * 6

        detection.confidence = self.safeJSONAccess(data, detection.confidence, 0.0)
        detection.id = self.safeJSONAccess(data, detection.id, 0)
        detection.name = self.safeJSONAccess(data, detection.name, "")
        detection.transformCameraPoseTargetSpace = self.safeJSONAccess(data, detection.keyTransformCameraPoseTargetSpace, default)
        detection.transformTargetPoseCameraSpace = self.safeJSONAccess(data, detection.keyTransformTargetPoseCameraSpace, default)
        detection.transformRobotPoseTargetSpace = self.safeJSONAccess(data, detection.keyTransformRobotPoseTargetSpace, default)
        detection.transformTargetPoseRobotSpace = self.safeJSONAccess(data, detection.keyTransformTargetPoseRobotSpace, default)
        detection.transformCameraPoseRobotSpace = self.safeJSONAccess(data, detection.keyTransformCameraPoseRobotSpace, default)
        detection.transformRobotPoseFieldSpace = self.safeJSONAccess(data, detection.keyTransformRobotPoseFieldSpace, default)
        detection.targetPixels.X() = self.safeJSONAccess(data, detection.keyTargetPixels.X(), 0.0)
        detection.targetPixels.Y() = self.safeJSONAccess(data, detection.keyTargetPixels.Y(), 0.0)
        detection.targetDegreesCrosshairAdjusted.X() = self.safeJSONAccess(data, detection.targetDegreesCrosshairAdjusted.X(), 0.0)
        detection.targetDegreesCrosshairAdjusted.Y() = self.safeJSONAccess(data, detection.targetDegreesCrosshairAdjusted.Y(), 0.0)
        detection.targetAreaNormalized = self.safeJSONAccess(data, detection.targetAreaNormalized, 0.0)
        detection.targetCorners = self.safeJSONAccess(data, detection.targetCorners, [])

    def fromJSON(self, data: Dict[str, Any], classification: ClassificationResults) -> None:
        default = [0.0] * 6

        classification.confidence = self.safeJSONAccess(data, classification.confidence, 0.0)
        classification.id = self.safeJSONAccess(data, classification.id, 0)
        classification.name = self.safeJSONAccess(data, classification.name, "")
        classification.transformCameraPoseTargetSpace = self.safeJSONAccess(data, classification.keyTransformCameraPoseTargetSpace, default)
        classification.transformTargetPoseCameraSpace = self.safeJSONAccess(data, classification.keyTransformTargetPoseCameraSpace, default)
        classification.transformRobotPoseTargetSpace = self.safeJSONAccess(data, classification.keyTransformRobotPoseTargetSpace, default)
        classification.transformTargetPoseRobotSpace = self.safeJSONAccess(data, classification.keyTransformTargetPoseRobotSpace, default)
        classification.transformCameraPoseRobotSpace = self.safeJSONAccess(data, classification.keyTransformCameraPoseRobotSpace, default)
        classification.transformRobotPoseFieldSpace = self.safeJSONAccess(data, classification.keyTransformRobotPoseFieldSpace, default)
        classification.targetPixels.X() = self.safeJSONAccess(data, classification.keyTargetPixels.X(), 0.0)
        classification.targetPixels.Y() = self.safeJSONAccess(data, classification.keyTargetPixels.Y(), 0.0)
        classification.targetDegreesCrosshairAdjusted.X() = self.safeJSONAccess(data, classification.targetDegreesCrosshairAdjusted.X(), 0.0)
        classification.targetDegreesCrosshairAdjusted.Y() = self.safeJSONAccess(data, classification.targetDegreesCrosshairAdjusted.Y(), 0.0)
        classification.targetAreaNormalized = self.safeJSONAccess(data, classification.targetAreaNormalized, 0.0)
        classification.targetCorners = self.safeJSONAccess(data, classification.targetCorners, [])

    def fromJSON(self, data: Dict[str, Any], vision: VisionResults) -> None:
        default = [0.0] * 6

        vision.timestamp = self.safeJSONAccess(data, vision.timestamp, 0.0)
        vision.latencyPipeline = self.safeJSONAccess(data, vision.latencyPipeline, 0.0)
        vision.latencyCapture = self.safeJSONAccess(data, vision.latencyCapture, 0.0)
        vision.pipelineIndex = self.safeJSONAccess(data, vision.pipelineIndex, 0.0)
        vision.valid = self.safeJSONAccess(data, vision.valid, 0.0)
        vision.robotPose = self.safeJSONAccess(data, vision.robotPose, default)
        vision.robotPoseBlue = self.safeJSONAccess(data, vision.robotPose, default)
        vision.robotPoseRed = self.safeJSONAccess(data, vision.robotPose, default)
        vision.retro = self.safeJSONAccess(data, vision.retro, [])
        vision.fiducial = self.safeJSONAccess(data, vision.fiducial, [])
        vision.detection = self.safeJSONAccess(data, vision.detection, [])
        vision.classification = self.safeJSONAccess(data, vision.classification, [])

    def fromJSON(self, data: Dict[str, Any], results: LimelightResults) -> None:
        results.targetingResults = self.safeJSONAccess(data, results.targetingResults, VisionResults())

    def getLatestResults(self, name: string, profile: bool = False) -> LimelightResults:
        start: float = time.perf_counter_ns()
        jsonStr: Any = getJSONDump(name)
        data: Dict[str, Any] = None

        try:
            jsonStr = getJSONDump(name)
            data = json.loads(jsonStr)

        except json.JSONDecodeError:
            return LimelightResults()

        end = time.perf_counter_ns()
        nanos = end - start
        millis = nanos * 1e-6

        try:
            out = LimelightResults()
            out.targetingResults.latencyJSON = millis
            if profile:
                print(f"lljson: {millis:.3f} ms")

            return out
        
        except Exception:
            return LimelightResults()

internal: Internal
