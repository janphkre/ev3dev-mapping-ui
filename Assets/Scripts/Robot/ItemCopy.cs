namespace ev3devMapping {

interface ItemCopy<T> {
    T DeepCopy();
    void copyFrom(T other);
}
}
