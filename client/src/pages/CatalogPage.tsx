// 상품 목록 페이지 — 냉장/냉동 필터, 검색, 주문
import { useEffect, useState } from 'react';
import { fetchProducts } from '../api/products';
import { createOrder } from '../api/orders';
import type { Product, StorageType } from '../types';
import { EMOJI_MAP } from '../constants';

interface Props {
  userId: number;
  refreshTrigger: number;      // App.tsx에서 올려주는 트리거 (WebSocket location_updated 시 재조회)
  onOrderCreated: () => void;  // 주문 성공 시 App.tsx에 알려 OrdersPage 재조회 트리거
}

type Filter = 'all' | StorageType;

export default function CatalogPage({ userId, refreshTrigger, onOrderCreated }: Props) {
  const [products, setProducts] = useState<Product[]>([]);
  const [filter, setFilter] = useState<Filter>('all');
  const [search, setSearch] = useState('');
  const [ordering, setOrdering] = useState<string | null>(null);  // 주문 중인 상품명 (중복 주문 방지)
  const [toast, setToast] = useState('');

  // refreshTrigger가 바뀔 때마다 상품 목록 재조회
  useEffect(() => {
    fetchProducts().then(setProducts);
  }, [refreshTrigger]);

  function showToast(msg: string) {
    setToast(msg);
    setTimeout(() => setToast(''), 2600);
  }

  async function handleOrder(product: Product) {
    if (product.stock <= 0 || ordering) return;
    setOrdering(product.name);
    try {
      await createOrder(userId, product.name);
      // 낙관적 업데이트: 서버 재조회 없이 즉시 재고 -1
      // 실패 시에는 다음 refreshTrigger에서 실제 재고로 복원됨
      setProducts((prev) =>
        prev.map((p) => p.name === product.name ? { ...p, stock: p.stock - 1 } : p)
      );
      onOrderCreated();
      showToast(`✅ ${product.name} 주문 완료`);
    } catch {
      showToast('주문에 실패했습니다. 다시 시도해 주세요.');
    } finally {
      setOrdering(null);
    }
  }

  const filtered = products.filter(
    (p) => (filter === 'all' || p.storage_type === filter) && p.name.includes(search),
  );

  return (
    <div>
      <h2 style={styles.h2}>상품 조회</h2>

      <div style={styles.toolbar}>
        <div style={styles.searchWrap}>
          <span style={styles.searchIcon}>🔍</span>
          <input
            style={styles.searchInput}
            placeholder="상품 검색"
            value={search}
            onChange={(e) => setSearch(e.target.value)}
          />
        </div>
        {(['all', 'chilled', 'frozen'] as Filter[]).map((f) => (
          <button
            key={f}
            style={{ ...styles.chip, ...(filter === f ? styles.chipOn : {}) }}
            onClick={() => setFilter(f)}
          >
            {f === 'all' ? '전체' : f === 'chilled' ? '❄️ 냉장' : '🧊 냉동'}
          </button>
        ))}
      </div>

      <div style={styles.grid}>
        {filtered.map((p) => {
          const soldOut = p.stock === 0;
          const isOrdering = ordering === p.name;
          return (
            <div key={p.name} style={styles.card}>
              <div style={styles.thumb}>{EMOJI_MAP[p.name] ?? '📦'}</div>
              <div style={styles.pname}>{p.name}</div>
              <span style={p.storage_type === 'chilled' ? styles.badgeChill : styles.badgeFrozen}>
                {p.storage_type === 'chilled' ? '❄️ 냉장 A' : '🧊 냉동 B'}
              </span>
              <div style={styles.stock}>
                {soldOut ? (
                  <span style={styles.soldout}>품절</span>
                ) : (
                  <>재고 <b>{p.stock}</b>개</>
                )}
              </div>
              {/* 품절 또는 주문 중이면 버튼 disabled */}
              <button
                style={soldOut || isOrdering ? styles.btnGhost : styles.btnOrder}
                disabled={soldOut || !!ordering}
                onClick={() => handleOrder(p)}
              >
                {isOrdering ? '주문 중...' : soldOut ? '품절' : '주문하기'}
              </button>
            </div>
          );
        })}
      </div>

      {toast && <div style={styles.toast}>{toast}</div>}
    </div>
  );
}

const styles: Record<string, React.CSSProperties> = {
  h2: { fontSize: 18, margin: '0 0 4px' },
  sub: { color: '#6b7280', fontSize: 13, margin: '0 0 20px' },
  toolbar: { display: 'flex', gap: 10, marginBottom: 18, flexWrap: 'wrap', alignItems: 'center' },
  searchWrap: { flex: 1, minWidth: 180, position: 'relative' },
  searchIcon: { position: 'absolute', left: 12, top: 9, fontSize: 14 },
  searchInput: {
    width: '100%', padding: '10px 13px 10px 36px',
    border: '1px solid #e5e7eb', borderRadius: 999, fontSize: 14, boxSizing: 'border-box',
  },
  chip: {
    padding: '8px 14px', border: '1px solid #e5e7eb', background: '#fff',
    borderRadius: 999, fontSize: 13, fontWeight: 600, color: '#6b7280', cursor: 'pointer',
  },
  chipOn: { background: '#dcfce7', color: '#15803d', borderColor: 'transparent' },
  grid: { display: 'grid', gridTemplateColumns: 'repeat(auto-fill, minmax(210px, 1fr))', gap: 16 },
  card: {
    background: '#fff', border: '1px solid #e5e7eb', borderRadius: 14,
    padding: 18, display: 'flex', flexDirection: 'column', gap: 10,
  },
  thumb: { fontSize: 42, textAlign: 'center', padding: '8px 0' },
  pname: { fontWeight: 700, fontSize: 16 },
  badgeChill: {
    display: 'inline-flex', alignItems: 'center', gap: 4,
    fontSize: 11, fontWeight: 700, padding: '3px 9px', borderRadius: 999,
    background: '#dbeafe', color: '#2563eb', width: 'fit-content',
  },
  badgeFrozen: {
    display: 'inline-flex', alignItems: 'center', gap: 4,
    fontSize: 11, fontWeight: 700, padding: '3px 9px', borderRadius: 999,
    background: '#cffafe', color: '#0891b2', width: 'fit-content',
  },
  stock: { fontSize: 13, color: '#6b7280' },
  soldout: { color: '#dc2626', fontWeight: 700 },
  btnOrder: {
    padding: '8px 16px', background: '#16a34a', color: '#fff',
    border: 'none', borderRadius: 10, fontSize: 13, fontWeight: 700, cursor: 'pointer',
  },
  btnGhost: {
    padding: '8px 16px', background: '#fff', color: '#6b7280',
    border: '1px solid #e5e7eb', borderRadius: 10, fontSize: 13, fontWeight: 700, cursor: 'default',
  },
  toast: {
    position: 'fixed', bottom: 24, left: '50%', transform: 'translateX(-50%)',
    background: '#1f2937', color: '#fff', padding: '12px 20px',
    borderRadius: 10, fontSize: 14, zIndex: 100,
  },
};
