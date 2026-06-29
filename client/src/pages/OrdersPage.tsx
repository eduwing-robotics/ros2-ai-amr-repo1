// 주문 목록 페이지 — 상태 배지, 4단계 스테퍼, 주문 취소
import { useEffect, useState } from 'react';
import { fetchMyOrders, cancelOrder } from '../api/orders';
import type { Order } from '../types';
import { EMOJI_MAP, ORDER_STATUS_KO, CANCEL_REASON_KO, ORDER_STEPS } from '../constants';

interface Props {
  userId: number;
  refreshTrigger: number;  // order_created/cancelled WebSocket 이벤트 시 재조회
}

export default function OrdersPage({ userId, refreshTrigger }: Props) {
  const [orders, setOrders] = useState<Order[]>([]);
  const [cancelling, setCancelling] = useState<number | null>(null);  // 취소 중인 주문 id
  const [toast, setToast] = useState('');

  // refreshTrigger 또는 userId가 바뀔 때마다 주문 목록 재조회
  useEffect(() => {
    fetchMyOrders(userId).then(setOrders);
  }, [userId, refreshTrigger]);

  function showToast(msg: string) {
    setToast(msg);
    setTimeout(() => setToast(''), 2600);
  }

  async function handleCancel(order: Order) {
    setCancelling(order.id);
    try {
      const updated = await cancelOrder(order.id);
      // 서버 응답으로 받은 업데이트된 주문으로 목록 즉시 갱신
      setOrders((prev) => prev.map((o) => (o.id === updated.id ? updated : o)));
      showToast(`주문 #${order.id} 취소됨 · 로봇이 상품을 창고로 되돌립니다`);
    } catch {
      showToast('취소에 실패했습니다.');
    } finally {
      setCancelling(null);
    }
  }

  if (orders.length === 0) {
    return <p style={{ textAlign: 'center', color: '#6b7280', padding: '60px 0' }}>주문 내역이 없습니다</p>;
  }

  return (
    <div>
      <h2 style={styles.h2}>내 주문</h2>
      <p style={styles.sub}>주문 상태를 실시간으로 확인하고, 출고 전이라면 취소할 수 있어요</p>

      {orders.map((order) => {
        const emoji = EMOJI_MAP[order.product_name] ?? '📦';
        const cancelled = order.status === 'cancelled';
        const stepIdx = ORDER_STEPS.findIndex((s) => s.status === order.status);
        const createdAt = new Date(order.created_at).toLocaleString('ko-KR', {
          month: 'numeric', day: 'numeric', hour: '2-digit', minute: '2-digit',
        });

        const statusStyle =
          order.status === 'processing' || order.status === 'pending' ? styles.sProc
          : order.status === 'awaiting_pickup' ? styles.sWait
          : order.status === 'delivered' ? styles.sDone
          : styles.sCancel;

        return (
          <div key={order.id} style={styles.card}>
            <div style={styles.head}>
              <div>
                <div style={styles.oid}>주문 #{order.id} · {createdAt}</div>
                <div style={styles.title}>{emoji} {order.product_name}</div>
              </div>
              <span style={{ ...styles.badge, ...statusStyle }}>{ORDER_STATUS_KO[order.status]}</span>
            </div>

            {/* 취소된 주문은 스테퍼 미표시 */}
            {!cancelled && (
              <div style={styles.stepper}>
                {ORDER_STEPS.map((step, i) => {
                  const done = i < stepIdx;
                  const cur = i === stepIdx;
                  return (
                    <div key={step.status} style={styles.step}>
                      <div style={{
                        ...styles.dot,
                        ...(done ? styles.dotDone : cur ? styles.dotCur : {}),
                      }}>
                        {done ? '✓' : i + 1}
                      </div>
                      <div style={{
                        ...styles.stepLabel,
                        ...(done || cur ? styles.stepLabelActive : {}),
                      }}>
                        {step.label}
                      </div>
                    </div>
                  );
                })}
              </div>
            )}

            {/* 수령 대기 시 게이트 안내 */}
            {order.status === 'awaiting_pickup' && (
              <div style={styles.pickup}>
                🚗 출고 게이트(OUT-1)에서 수령하세요 · 수령번호 <b>#{order.id}</b>
                <br />
                <span style={{ fontSize: 12 }}>5분 내 미수령 시 자동 취소됩니다</span>
              </div>
            )}

            {/* 취소 사유 표시 */}
            {cancelled && order.cancel_reason && (
              <p style={styles.cancelMsg}>
                ⛔ 취소 사유: {CANCEL_REASON_KO[order.cancel_reason]}
              </p>
            )}

            <div style={styles.foot}>
              <span style={styles.oid}>{cancelled ? '종료된 주문' : ''}</span>
              {/* pending/processing 상태만 취소 버튼 표시 — awaiting_pickup 이후는 불가 */}
              {(order.status === 'pending' || order.status === 'processing') && (
                <button
                  style={styles.btnCancel}
                  disabled={cancelling === order.id}
                  onClick={() => handleCancel(order)}
                >
                  {cancelling === order.id ? '취소 중...' : '주문 취소'}
                </button>
              )}
              {order.status === 'delivered' && (
                <span style={styles.oid}>이용해 주셔서 감사합니다 🙏</span>
              )}
            </div>
          </div>
        );
      })}

      {toast && <div style={styles.toast}>{toast}</div>}
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  h2: { fontSize: 18, margin: '0 0 4px' },
  sub: { color: '#6b7280', fontSize: 13, margin: '0 0 20px' },
  card: {
    background: '#fff', border: '1px solid #e5e7eb', borderRadius: 14,
    padding: 20, marginBottom: 16,
  },
  head: { display: 'flex', alignItems: 'center', justifyContent: 'space-between', marginBottom: 16 },
  oid: { fontSize: 12, color: '#6b7280' },
  title: { fontSize: 17, fontWeight: 700, marginTop: 2 },
  badge: { fontSize: 12, fontWeight: 800, padding: '5px 12px', borderRadius: 999 },
  sProc: { background: '#dcfce7', color: '#15803d' },
  sWait: { background: '#fef3c7', color: '#d97706' },
  sDone: { background: '#e5e7eb', color: '#374151' },
  sCancel: { background: '#fee2e2', color: '#dc2626' },
  stepper: { display: 'flex', alignItems: 'flex-start', margin: '8px 0 4px' },
  step: { flex: 1, textAlign: 'center', position: 'relative' },
  dot: {
    width: 26, height: 26, borderRadius: '50%', background: '#e5e7eb', color: '#9ca3af',
    display: 'flex', alignItems: 'center', justifyContent: 'center',
    fontSize: 13, fontWeight: 700, margin: '0 auto', position: 'relative', zIndex: 1,
  },
  dotDone: { background: '#16a34a', color: '#fff' },
  dotCur: { background: '#d97706', color: '#fff', boxShadow: '0 0 0 4px #fef3c7' },
  stepLabel: { fontSize: 11, color: '#6b7280', marginTop: 6 },
  stepLabelActive: { color: '#1f2937', fontWeight: 600 },
  pickup: {
    background: '#fef3c7', border: '1px solid #fde68a', borderRadius: 10,
    padding: '12px 14px', marginTop: 14, fontSize: 13, color: '#92400e',
  },
  cancelMsg: { fontSize: 13, color: '#dc2626', margin: '10px 0 0' },
  foot: {
    display: 'flex', alignItems: 'center', justifyContent: 'space-between',
    marginTop: 16, paddingTop: 14, borderTop: '1px dashed #e5e7eb',
  },
  btnCancel: {
    padding: '8px 16px', background: '#fff', color: '#6b7280',
    border: '1px solid #e5e7eb', borderRadius: 10, fontSize: 13, fontWeight: 700, cursor: 'pointer',
  },
  toast: {
    position: 'fixed', bottom: 24, left: '50%', transform: 'translateX(-50%)',
    background: '#1f2937', color: '#fff', padding: '12px 20px',
    borderRadius: 10, fontSize: 14, zIndex: 100,
  },
};
