plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
    id("org.jetbrains.kotlin.plugin.compose")
}

android {
    namespace = "id.leumpang.mata"
    compileSdk = 35
    defaultConfig {
        applicationId = "id.leumpang.mata"
        minSdk = 26
        targetSdk = 35
        versionCode = 1
        versionName = "0.3.0"
    }
    buildTypes {
        release { isMinifyEnabled = false }
    }
    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }
    kotlinOptions { jvmTarget = "17" }
    buildFeatures { compose = true }
    packaging { resources { excludes += setOf("META-INF/*.kotlin_module", "META-INF/INDEX.LIST", "META-INF/DEPENDENCIES") } }
}

dependencies {
    implementation(project(":lap"))
    implementation(project(":bus"))
    implementation(project(":perception"))

    implementation("androidx.core:core-ktx:1.13.1")
    implementation("androidx.activity:activity-compose:1.9.3")
    implementation("androidx.lifecycle:lifecycle-runtime-ktx:2.8.7")
    implementation(platform("androidx.compose:compose-bom:2024.10.01"))
    implementation("androidx.compose.ui:ui")
    implementation("androidx.compose.material3:material3")
    implementation("androidx.compose.material:material-icons-core")

    val camerax = "1.4.0"
    implementation("androidx.camera:camera-core:$camerax")
    implementation("androidx.camera:camera-camera2:$camerax")
    implementation("androidx.camera:camera-lifecycle:$camerax")

    implementation("com.google.mediapipe:tasks-vision:0.10.14")
    implementation("com.github.mik3y:usb-serial-for-android:3.8.1")
    implementation("org.java-websocket:Java-WebSocket:1.5.7")
}
