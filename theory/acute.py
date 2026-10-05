# 出所: 数学の壁打ち会話(2026-10-03)の作業環境で書いた計算スクリプト。シミュレータの出力ではない。
# 内容: 学習型の担当計算機を複数並べたときの「理想化した1ステップの写像」の更新行列を作り、
#       スペクトル半径が1を超える r(安定限界)を分割数ごとに二分法で求める。結果は learned_theory.dat。
# 理想化の仮定: 最適化が毎ステップ完全に収束する。境目の両側の格子点は前進オイラー法、
#       内部は境目の新しい値を境界値とする後退オイラー法を厳密に解く。区間全体の両端は温度0。
# 格子: 65点(格子間隔1/64)。分割位置は格子点番号で、2分割=[32]、4分割=[16,32,48] など。
import numpy as np
n=65
def make_E(splits):
    E=set()
    for s in splits: E.add(s); E.add(s+1)
    return E
def step_fn(r,E):
    def step(u):
        w=u.copy()
        for p in E: w[p]=u[p]+r*(u[p-1]-2*u[p]+u[p+1])
        new=w.copy(); new[0]=0; new[n-1]=0
        i=1
        while i<n-1:
            if i in E: i+=1; continue
            j=i
            while j<n-1 and j not in E: j+=1
            m=j-i
            A=np.diag((1+2*r)*np.ones(m))+np.diag(-r*np.ones(m-1),1)+np.diag(-r*np.ones(m-1),-1)
            rhs=u[i:j].copy()
            if (i-1) in E: rhs[0]+=r*w[i-1]
            if j in E: rhs[-1]+=r*w[j]
            new[i:j]=np.linalg.solve(A,rhs); i=j
        return new
    return step
def mat(r,splits):
    st=step_fn(r,make_E(splits)); M=np.zeros((n-2,n-2))
    for c in range(1,n-1):
        u=np.zeros(n); u[c]=1; M[:,c-1]=st(u)[1:n-1]
    return M
def rho(r,splits): return np.max(np.abs(np.linalg.eigvals(mat(r,splits))))
def rstar(splits):
    lo,hi=0.3,3.0
    for _ in range(45):
        mid=0.5*(lo+hi)
        if rho(mid,splits)>1+1e-12: hi=mid
        else: lo=mid
    return 0.5*(lo+hi)
cases={2:[32],4:[16,32,48],8:list(range(8,64,8)),16:list(range(4,64,4)),32:list(range(2,64,2))}
if __name__=='__main__':
    print('閉じた式 (7-sqrt17)/4 =', (7-np.sqrt(17))/4)
    with open('learned_theory.dat','w') as f:
        f.write('# 分割数  理論の安定限界r\n')
        for N,sp in cases.items():
            rs=rstar(sp); print(N, f'{rs:.4f}'); f.write(f'{N} {rs:.4f}\n')
    # 1台(分割なし)
    print('1台: r=5.0でのスペクトル半径', f'{rho(5.0,[]):.6f}')
