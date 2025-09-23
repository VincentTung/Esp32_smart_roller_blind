package com.vincent.library.base.ui

import android.os.Bundle
import android.view.View
import android.widget.ImageView
import android.widget.TextView
import androidx.fragment.app.FragmentActivity
import com.loading.dialog.IOSLoadingDialog
import com.vincent.library.base.R
import com.vincent.library.base.util.ToastUtil
import com.vincent.library.base.util.logd

/**
 * UI基类
 */
open class VTBaseActivity : FragmentActivity() {
    var iosLoadingDialog: IOSLoadingDialog? = null

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
    }

    /**
     * 初始化标题内容和返回键
     */
    fun initTitle(stringId: Int, isBackVisible: Boolean = true) {
        findViewById<TextView>(R.id.tv_title).text = getString(stringId)
        findViewById<ImageView>(R.id.iv_back).let {

            if (isBackVisible) {
                it.visibility = View.VISIBLE
                it.setOnClickListener {
                    finish()
                }
            } else {
                it.visibility = View.GONE
            }
        }
    }


    fun startLoading(text: String) {
        logd("startLoading: ")
        stopLoading()
        iosLoadingDialog = IOSLoadingDialog().setHintMsg(text).setOnTouchOutside(false)
        iosLoadingDialog!!.show(fragmentManager, text)
    }

    fun updateLoadingProgress(progress: Int) {
        // 更新loading文本显示进度
        val progressText = "发送GIF中... $progress%"
        logd("updateLoadingProgress: $progressText")
        iosLoadingDialog?.setHintMsg(progressText)
    }

    fun stopLoading() {
        logd("stopLoading: ")
        try {
            if (!isFinishing && !isDestroyed && !supportFragmentManager.isStateSaved) {
                iosLoadingDialog?.dismiss()
            }
        } catch (e: Exception) {
            logd("stopLoading异常: ${e.message}")
        } finally {
            iosLoadingDialog = null
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        // 确保在Activity销毁时清理对话框
        try {
            if (!supportFragmentManager.isStateSaved) {
                iosLoadingDialog?.dismiss()
            }
        } catch (e: Exception) {
            logd("onDestroy中stopLoading异常: ${e.message}")
        } finally {
            iosLoadingDialog = null
        }
        ToastUtil.clear()
    }
}