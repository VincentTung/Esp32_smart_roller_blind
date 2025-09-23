package com.vincent.android.rollerblindcontroller.app

import android.app.Application
import com.vincent.android.rollerblindcontroller.logic.RollerBlindController
import com.vincent.library.base.util.MMKVUtil

import com.vincent.library.ble.logic.BLEDeviceManager

class RollerBlindApp: Application() {

    companion object {
        private const val TAG = "RollerBlindApp"
    }

    override fun onCreate() {
        super.onCreate()
        MMKVUtil.init(this)
        BLEDeviceManager.init(this)
        RollerBlindController.getInstance().initBLE(this)
    }
    

}