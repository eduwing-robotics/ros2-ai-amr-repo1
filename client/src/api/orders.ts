import api from './client';
import type { Order } from '../types';

// GET /orders/me?user_id= — 내 주문 목록 (최신순)
export async function fetchMyOrders(userId: number): Promise<Order[]> {
  const res = await api.get<Order[]>('/orders/me', { params: { user_id: userId } });
  return res.data;
}

// POST /orders — 주문 생성, 서버에서 NOTIFY new_order 발행 → Task Manager 수신
export async function createOrder(userId: number, productName: string): Promise<Order> {
  const res = await api.post<Order>('/orders', { user_id: userId, product_name: productName });
  return res.data;
}

// POST /orders/:id/cancel — 주문 취소 (pending/processing만 가능, 그 외 서버에서 409)
export async function cancelOrder(orderId: number): Promise<Order> {
  const res = await api.post<Order>(`/orders/${orderId}/cancel`);
  return res.data;
}
