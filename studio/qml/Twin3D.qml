import QtQuick
import QtQuick.Controls
import QtQuick.Layouts
import QtQuick3D
import QtQuick3D.Helpers
import Leumpang

Item {
    id: root

    View3D {
        id: view
        anchors.fill: parent
        camera: cam

        environment: SceneEnvironment {
            backgroundMode: SceneEnvironment.Color
            clearColor: "#191d22"
            antialiasingMode: SceneEnvironment.MSAA
            antialiasingQuality: SceneEnvironment.High
            aoEnabled: true
            aoStrength: 45
            aoDistance: 12
            aoSoftness: 40
            tonemapMode: SceneEnvironment.TonemapModeFilmic
        }

        Node {
            id: orbit
            position: Qt.vector3d(0, 190, 0)
            eulerRotation.x: -12
            eulerRotation.y: 28
            PerspectiveCamera {
                id: cam
                position: Qt.vector3d(0, 0, 980)
                clipNear: 5
                clipFar: 20000
                fieldOfView: 36
            }
        }

        DirectionalLight {
            eulerRotation.x: -52
            eulerRotation.y: 32
            brightness: 1.5
            castsShadow: true
            shadowMapQuality: Light.ShadowMapQualityVeryHigh
            shadowFactor: 60
        }
        DirectionalLight {
            eulerRotation.x: -18
            eulerRotation.y: -145
            brightness: 0.55
            color: "#cfe0ff"
        }
        DirectionalLight {
            eulerRotation.x: 60
            brightness: 0.18
            color: "#ffe2c4"
        }

        // lantai + grid 50 mm
        Model {
            source: "#Rectangle"
            scale: Qt.vector3d(40, 40, 1)
            eulerRotation.x: -90
            receivesShadows: true
            materials: PrincipledMaterial { baseColor: "#2a2f36"; roughness: 0.92; metalness: 0 }
        }
        Model {
            y: 0.4
            eulerRotation.x: -90
            castsShadows: false
            geometry: GridGeometry {
                horizontalLines: 41
                verticalLines: 41
                horizontalStep: 50
                verticalStep: 50
            }
            materials: PrincipledMaterial { baseColor: "#3c4550"; lighting: PrincipledMaterial.NoLighting }
        }

        // robot (pose utama: robot atau pratinjau)
        Repeater3D {
            model: twin.meshes
            delegate: Model {
                required property var modelData
                geometry: StlMesh { source: modelData.source }
                position: twin.positions[modelData.link]
                rotation: twin.rotations[modelData.link]
                visible: modelData.link !== twin.phoneLink || twin.showPhone
                castsShadows: true
                receivesShadows: true
                materials: PrincipledMaterial {
                    baseColor: modelData.color
                    metalness: modelData.metalness
                    roughness: modelData.roughness
                    emissiveFactor: modelData.link === twin.highlightLink ? Qt.vector3d(0.45, 0.2, 0.02) : Qt.vector3d(0, 0, 0)
                }
            }
        }

        // hantu: pose robot (saat pratinjau) atau umpan balik servo aktual
        Repeater3D {
            model: twin.meshes
            delegate: Model {
                required property var modelData
                geometry: StlMesh { source: modelData.source }
                position: twin.ghostPositions[modelData.link]
                rotation: twin.ghostRotations[modelData.link]
                visible: twin.ghostVisible && modelData.link !== twin.phoneLink
                castsShadows: false
                receivesShadows: false
                materials: PrincipledMaterial {
                    baseColor: "#4fc3f7"
                    opacity: 0.2
                    alphaMode: PrincipledMaterial.Blend
                    lighting: PrincipledMaterial.NoLighting
                    depthDrawMode: Material.NeverDepthDraw
                }
            }
        }
    }

    OrbitCameraController {
        anchors.fill: parent
        origin: orbit
        camera: cam
    }

    // ---- hamparan ----
    ColumnLayout {
        anchors.left: parent.left
        anchors.top: parent.top
        anchors.margins: 12
        spacing: 4
        Label {
            text: twin.previewOwner.length > 0 ? "PRATINJAU: " + twin.previewOwner : (robot.connected ? "ROBOT (perintah sendi)" : "MODEL (HOME)")
            font.bold: true
            color: twin.previewOwner.length > 0 ? "#E86A1C" : "#9fb3c8"
        }
        Label {
            text: twin.previewOwner.length > 0 ? "biru transparan = pose robot" : "biru transparan = umpan balik servo aktual"
            opacity: 0.6
            font.pixelSize: 12
            visible: twin.ghostVisible
        }
    }
    RowLayout {
        anchors.left: parent.left
        anchors.bottom: parent.bottom
        anchors.margins: 8
        spacing: 2
        CheckBox { text: "Hantu"; checked: twin.ghostVisible; onToggled: twin.ghostVisible = checked }
        CheckBox { text: "HP di punggung"; checked: twin.showPhone; onToggled: twin.showPhone = checked }
        CheckBox { text: "Miring IMU"; checked: twin.imuTilt; onToggled: twin.imuTilt = checked }
        Button {
            text: "Kembali ke robot"
            visible: twin.previewOwner.length > 0
            onClicked: twin.clearPreview("")
        }
        Button {
            text: "Kamera awal"
            flat: true
            onClicked: { orbit.eulerRotation = Qt.vector3d(-12, 28, 0); cam.position = Qt.vector3d(0, 0, 980) }
        }
    }
}
