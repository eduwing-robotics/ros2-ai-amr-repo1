import api from './client';
import type { Product } from '../types';

// GET /products — 전체 상품 목록 + 실시간 재고 수량 조회
// stock은 reserved_by IS NULL인 슬롯 수 (서버에서 집계)
export async function fetchProducts(): Promise<Product[]> {
  const res = await api.get<Product[]>('/products');
  return res.data;
}
