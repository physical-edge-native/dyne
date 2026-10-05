# 出所: 数学の壁打ち会話(2026-10-03)の作業環境で書いた計算スクリプト。シミュレータの出力ではない。
# 内容: 左端が透過境界条件、右端が温度0の差分法(前進オイラー法)について、温度と補助変数をまとめた
#       状態の更新行列を作り、スペクトル半径を求める。結果は transparent_rho.dat。
# 実装と照合すべき仮定:
#   (1) 境目の更新式 u0_new = u0 + 2r(u1 - u0) - (2rh/alpha) q (q は外へ流れ出る熱流束)
#   (2) 半陰的方式: 当ステップの境目の温度変化に係数 kappa を掛けた項を陰的に解く
#       D = (2r(u1-u0) - K*Ahist) / (1 + K*kappa)、K = 2rh/alpha
#   (3) 補助変数の更新 y_j <- exp(-lambda_j dt) y_j + exp(-lambda_j dt/2) * (境目の温度の変化)
#   (4) 指数和の節点と重み: 対数軸上の台形公式(刻み0.35、時間範囲0.2)。項数は55〜56。実機は25項で係数も異なる
#   (5) alpha=1、格子65点。方式 semi=半陰的、delay=1ステップ遅延、drop=直近項を落とす
import numpy as np
n=65; alpha=1.0; h=1.0/(n-1)

def soe(dt, Tmax, hq=0.35, c1=1e9, c2=2e4):
    xlo=0.5*np.log(1/(c1*Tmax)); xhi=0.5*np.log(c2/dt)
    xs=np.arange(xlo,xhi,hq)
    return np.exp(2*xs),(2/np.sqrt(np.pi))*hq*np.exp(xs)

def update_matrix(r, mode, Tmax=0.2):
    dt=r*h*h/alpha
    lam,w=soe(dt,Tmax); M=len(lam)
    c=np.sqrt(alpha/np.pi); dec=np.exp(-lam*dt); half=np.exp(-lam*dt/2)
    kap=c*np.sum(w*half); K=2*r*h/alpha
    nu=n-1  # u_0..u_{n-2} (u_{n-1}=0 固定)
    S=nu+M
    def step(s):
        u=np.zeros(n); u[:nu]=s[:nu]; y=s[nu:]
        new=u.copy()
        new[1:n-1]=u[1:n-1]+r*(u[:-2]-2*u[1:n-1]+u[2:])
        new[n-1]=0.0
        Ahist=c*np.sum(w*dec*y); qprev=c*np.sum(w*y)
        if mode=='semi': D=(2*r*(u[1]-u[0])-K*Ahist)/(1+K*kap)
        elif mode=='delay': D=2*r*(u[1]-u[0])-K*qprev
        elif mode=='drop': D=2*r*(u[1]-u[0])-K*Ahist
        new[0]=u[0]+D
        y2=dec*y+half*D
        return np.concatenate([new[:nu],y2])
    G=np.zeros((S,S))
    for i in range(S):
        e=np.zeros(S); e[i]=1.0; G[:,i]=step(e)
    return G,M

def rho(r,mode):
    G,M=update_matrix(r,mode)
    return np.max(np.abs(np.linalg.eigvals(G))),M

if __name__=='__main__':
    for mode in ['semi','delay','drop']:
        print('---',mode)
        for r in [0.30,0.32,0.34,0.35,0.40,0.45,0.49,0.50,0.51,0.55]:
            p,M=rho(r,mode)
            print(f'r={r:.2f}  ρ={p:.6f}  補助変数{M}個  {"不安定" if p>1+1e-9 else "安定"}')
