# 出所: 数学の壁打ち会話(2026-10-03)の作業環境で書いた計算スクリプト。シミュレータの出力ではない。
# 内容: 差分法(前進オイラー法、両端温度0)に、観測点でのナッジング u <- u + gamma(y - u) を毎ステップ加えたときの
#       誤差の更新行列 (I - gamma P) G のスペクトル半径を求め、r ごとの安定境界 gamma を計算する。
#       結果は nudging_boundary.dat と nudging_cells.dat のスペクトル半径の列。
# 実装と照合すべき仮定:
#   (1) 観測点の格子点番号: 実装は 8, 25, 41, 57(2026-10-04 照合済み。dyne-sim の分割は 65点を
#       17,16,16,16 に割り(余り1を先頭ノードへ)、観測点 = 区間開始 + 点数/2。当初の仮定 8,24,40,56 から修正)
#   (2) 1ステップの順序は「差分法で計算 → ナッジング」
#   (3) 格子65点、両端温度0、観測周期1
import numpy as np
n=65
obs=[8,25,41,57]  # 実装照合済み (旧仮定: [8,24,40,56])
def G(r):
    m=n-2; A=np.zeros((m,m))
    for i in range(m):
        A[i,i]=1-2*r
        if i>0: A[i,i-1]=r
        if i<m-1: A[i,i+1]=r
    return A
P=np.zeros((n-2,n-2))
for p in obs: P[p-1,p-1]=1
def rho(r,g): return np.max(np.abs(np.linalg.eigvals((np.eye(n-2)-g*P)@G(r))))
def gstar(r,gmax=12.0):
    prev=2.0
    for g in np.arange(2.0,gmax,0.02):
        if rho(r,g)>1+1e-12:
            lo,hi=prev,g
            for _ in range(25):
                mid=0.5*(lo+hi)
                if rho(r,mid)>1+1e-12: hi=mid
                else: lo=mid
            return 0.5*(lo+hi)
        prev=g
    return np.nan
if __name__=='__main__':
    rs=np.round(np.arange(0.02,0.501,0.01),3)
    with open('nudging_boundary.dat','w') as f:
        f.write('# r  安定境界gamma(固有値計算)  局所近似1+1/(1-2r)\n')
        for r in rs:
            gs=gstar(r); simple=1+1/(1-2*r) if r<0.5 else np.nan
            f.write(f'{r:.3f}  {gs:.4f}  {simple:.4f}\n')
            print(f'{r:.3f}  {gs:.4f}  {simple:.4f}')
