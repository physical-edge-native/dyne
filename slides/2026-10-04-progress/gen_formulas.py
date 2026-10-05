# 式画像の生成 (matplotlib mathtext、背景透明、本文色=黒)
# 各画像の元の式文字列はスライドの発表者ノートにも records として残す
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import os

OUT = "formula_imgs"
os.makedirs(OUT, exist_ok=True)

FORMULAS = {
 "f_heat":   r"$\dfrac{\partial u}{\partial t} = \alpha\,\dfrac{\partial^2 u}{\partial x^2}$",
 "f_fdm":    r"$u_j^{m+1} = u_j^m + r\,(u_{j-1}^m - 2u_j^m + u_{j+1}^m)$",
 "f_r":      r"$r = \alpha\,\Delta t\,/\,h^2$",
 "f_nn":     r"$u_\theta(x) = \sum_{k=1}^{16} c_k\,\tanh(a_k x + b_k) + e$",
 "f_resid":  r"$R(x) = \dfrac{u_\theta(x) - u_{\mathrm{prev}}(x)}{\Delta t} - \alpha\,u_\theta''(x)$",
 "f_extrap": r"$u_{-1} = 2u_0 - u_1 \;\;\Rightarrow\;\; u_0^{m+1} = u_0^m$",
 "f_dtn":    r"$-\dfrac{\partial u}{\partial x}(0,t) = \dfrac{1}{\sqrt{\pi\alpha}}\int_0^t \dfrac{g'(\tau)}{\sqrt{t-\tau}}\,d\tau$",
 "f_beta":   r"$\beta(t) = 1\,/\,\sqrt{\pi\alpha t}$",
 "f_bucket": r"$y_j^{m+1} = e^{-\lambda_j \Delta t}\,y_j^m + e^{-\lambda_j \Delta t/2}\,\Delta g^m$",
 "f_soe":    r"$\dfrac{1}{\sqrt{t}} \approx \sum_{j=1}^{M} w_j\,e^{-\lambda_j t}$",
 "f_nudge":  r"$u \;\leftarrow\; u + \gamma\,(y - u)$",
 "f_eig":    r"$\lambda_k = -\dfrac{4\alpha}{h^2}\,\sin^2\!\left(\dfrac{k\pi h}{2}\right)$",
 "f_phi_fe": r"$\phi(z) = 1 + z$",
 "f_phi_be": r"$\phi(z) = \dfrac{1}{1 - z}$",
 "f_limit":  r"$r^{*} = \dfrac{7-\sqrt{17}}{4} \approx 0.719$",
 "f_quad":   r"$2r^2 - 7r + 4 = 0$",
 "f_gp":     r"$e^{m+1} = (I - \gamma P)\,G\,e^m$",
 "f_gamma2": r"$r \leq 1/2,\;\; 0 \leq \gamma \leq 2$",
 "f_target": r"$u_p^{\mathrm{new}} = u_p + r\,(u_{p-1} - 2u_p + u_{p+1})$",
 "f_rownorm": r"$\Vert (I-\gamma P)G \Vert_\infty \leq \Vert I-\gamma P \Vert_\infty\,\Vert G \Vert_\infty \leq 1$",
 "f_local":  r"$\gamma \leq 1 + \dfrac{1}{1-2r}$",
 "f_mu":     r"$\mu = \dfrac{r}{4-r} \approx 0.219$",
 "f_erfc":   r"$u(x,t) = U_0\,\mathrm{erfc}\!\left(\dfrac{x}{2\sqrt{\alpha t}}\right)$",
}

for name, tex in FORMULAS.items():
    fig = plt.figure(figsize=(0.1, 0.1))
    t = fig.text(0, 0, tex, fontsize=26, color="#000000")
    fig.savefig(os.path.join(OUT, name + ".png"), dpi=200, transparent=True,
                bbox_inches="tight", pad_inches=0.06)
    plt.close(fig)
    print(name)

# 式文字列の控え (ノート転記用)
with open(os.path.join(OUT, "formulas.txt"), "w", encoding="utf-8") as f:
    for name, tex in FORMULAS.items():
        f.write(f"{name}: {tex}\n")
print("done")
