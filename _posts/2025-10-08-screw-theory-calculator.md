---
title: "旋量指数映射在线计算器"
date: 2025-10-08 12:00:00 +0800
category: 机器人学
tags: [旋量理论, 计算器, 工具, 指数映射, SE(3)]
author: Bathelor
toc: false
math: true
head_scripts:
  - https://cdnjs.cloudflare.com/ajax/libs/mathjs/12.3.0/math.js
---

在上一篇[《掌握现代机器人学：基于旋量理论的正运动学》](/posts/robot-kinematics-with-screw-theory/)教程中，我们详细探讨了指数映射 $e^{[\mathcal{S}]\theta}$ 的理论和计算方法。它是将一个螺旋轴 $S$ 和关节变量 $\theta$ 转换为一个 $SE(3)$ 空间下4x4齐次变换矩阵的关键工具。

为了方便学习和验证计算，本页提供一个基于 JavaScript 的在线计算器。您只需输入6维螺旋轴 $S=(\omega, v)$ 和关节变量 $\theta$，即可实时计算出对应的变换矩阵。

<div id="screw-calculator" style="background-color: #2d2d2d; padding: 20px; border-radius: 8px; color: #e0e0e0;">

  <h3>旋量指数映射计算器</h3>

  <div style="margin-bottom: 20px;">
    <label style="display: block; margin-bottom: 5px;"><strong>螺旋轴 S = (ω, v)</strong></label>
    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px; margin-bottom: 10px;">
      <input type="number" id="w_x" placeholder="ω_x" value="0" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
      <input type="number" id="w_y" placeholder="ω_y" value="0" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
      <input type="number" id="w_z" placeholder="ω_z" value="1" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
    </div>
    <div style="display: grid; grid-template-columns: repeat(3, 1fr); gap: 10px;">
      <input type="number" id="v_x" placeholder="v_x" value="0" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
      <input type="number" id="v_y" placeholder="v_y" value="0" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
      <input type="number" id="v_z" placeholder="v_z" value="0.3183" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
    </div>
  </div>

  <div style="margin-bottom: 20px;">
    <label style="display: block; margin-bottom: 5px;"><strong>关节变量 θ</strong></label>
    <input type="number" id="theta" value="3.14159" style="width: 100%; box-sizing: border-box; padding: 8px; background-color: #3c3c3c; border: 1px solid #555; color: #fff; border-radius: 4px;">
    <div style="margin-top: 10px;">
      <input type="radio" id="radians" name="angle_unit" value="rad" checked>
      <label for="radians">弧度制 (Radians)</label>
      <input type="radio" id="degrees" name="angle_unit" value="deg" style="margin-left: 20px;">
      <label for="degrees">角度制 (Degrees)</label>
    </div>
  </div>

  <button id="calculate-btn" style="width: 100%; padding: 10px; background-color: #007bff; color: white; border: none; border-radius: 4px; cursor: pointer; font-size: 16px;">计算 SE(3) 矩阵</button>

  <div style="margin-top: 20px;">
    <label style="display: block; margin-bottom: 5px;"><strong>结果 T = e<sup>[S]θ</sup></strong></label>
    <pre id="result-matrix" style="background-color: #1e1e1e; padding: 15px; border-radius: 4px; font-family: 'Courier New', Courier, monospace; font-size: 14px; white-space: pre; line-height: 1.5;"></pre>
  </div>
</div>

<script>
  // 等待整个页面的HTML都加载完毕后再执行脚本
  document.addEventListener('DOMContentLoaded', function() {
    console.log("【调试】: 页面DOM加载完成，脚本开始执行。");

    const calculateBtn = document.getElementById('calculate-btn');
    const resultMatrixEl = document.getElementById('result-matrix');

    if (!calculateBtn || !resultMatrixEl) {
      console.error("【调试】: 致命错误！计算器核心HTML元素未找到。");
      return;
    }
    console.log("【调试】: 计算按钮和结果区域已成功找到。");

    // 绑定点击事件
    calculateBtn.addEventListener('click', function() {
      console.log("【调试】: 计算按钮被点击。");

      // 检查math.js库是否已加载
      if (typeof math === 'undefined') {
        console.error("【调试】: 错误！math.js 在点击时仍未定义。");
        resultMatrixEl.innerText = "错误：Math.js 数学库加载失败。\n请检查网络连接或 head_scripts 配置。";
        return;
      }
      console.log("【调试】: math.js 已定义，开始获取输入。");

      // 1. 获取所有输入值
      const w_x = parseFloat(document.getElementById('w_x').value);
      const w_y = parseFloat(document.getElementById('w_y').value);
      const w_z = parseFloat(document.getElementById('w_z').value);
      const v_x = parseFloat(document.getElementById('v_x').value);
      const v_y = parseFloat(document.getElementById('v_y').value);
      const v_z = parseFloat(document.getElementById('v_z').value);
      let theta = parseFloat(document.getElementById('theta').value);
      const unit = document.querySelector('input[name="angle_unit"]:checked').value;
      
      console.log("【调试】: 输入值已获取: ", {w_x, w_y, w_z, v_x, v_y, v_z, theta, unit});

      if (isNaN(w_x) || isNaN(w_y) || isNaN(w_z) || isNaN(v_x) || isNaN(v_y) || isNaN(v_z) || isNaN(theta)) {
          console.error("【调试】: 错误！输入值包含非数字。");
          resultMatrixEl.innerText = "错误：所有输入都必须是有效的数字。";
          return;
      }

      // 2. 角度制转换为弧度制
      if (unit === 'deg') {
          theta = theta * Math.PI / 180;
          console.log("【调试】: 角度已转换为弧度: ", theta);
      }

      // 3. 创建向量和矩阵
      const w = [w_x, w_y, w_z];
      const v = [v_x, v_y, v_z];
      
      console.log("【调试】: 开始计算 R 和 p...");
      // ...后续计算代码...
      const I = math.identity(3);
      const w_skew = math.matrix([[0, -w_z, w_y], [w_z, 0, -w_x], [-w_y, w_x, 0]]);
      const w_skew_sq = math.multiply(w_skew, w_skew);

      const term_sin = math.multiply(w_skew, Math.sin(theta));
      const term_cos = math.multiply(w_skew_sq, (1 - Math.cos(theta)));
      const R = math.add(I, term_sin, term_cos);
      console.log("【调试】: 旋转矩阵 R 计算完成。");

      const term1_p = math.multiply(I, theta);
      const term2_p = math.multiply(w_skew, (1 - Math.cos(theta)));
      const term3_p = math.multiply(w_skew_sq, (theta - Math.sin(theta)));
      const p_factor_matrix = math.add(term1_p, term2_p, term3_p);
      const p = math.multiply(p_factor_matrix, v);
      console.log("【调试】: 平移向量 p 计算完成。");

      const T = math.matrix([
          [R.get([0, 0]), R.get([0, 1]), R.get([0, 2]), p.get([0])],
          [R.get([1, 0]), R.get([1, 1]), R.get([1, 2]), p.get([1])],
          [R.get([2, 0]), R.get([2, 1]), R.get([2, 2]), p.get([2])],
          [0, 0, 0, 1]
      ]);
      
      console.log("【调试】: 准备显示最终矩阵。");
      displayMatrix(T);
    });

    function displayMatrix(matrix) {
      let matrixString = "";
      for (let i = 0; i < 4; i++) {
          let row = [];
          for (let j = 0; j < 4; j++) {
              row.push(matrix.get([i, j]).toFixed(4).padStart(9, ' '));
          }
          matrixString += row.join("  ");
          if (i < 3) {
              matrixString += "\n";
          }
      }
      resultMatrixEl.innerText = matrixString;
      console.log("【调试】: 矩阵已更新到页面。");
    }
    
    // 移除自动加载功能，仅依赖手动点击，以简化调试
    resultMatrixEl.innerText = "请点击上方按钮进行计算。";

  });
</script>
