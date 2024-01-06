/**
 * Util tuple class due to lack of javafx
 * Don't need it right now, might be useful
 */
package org.firstinspires.ftc.teamcode.util;

public class Pair<K, V> {
    private final K key;
    private final V value;

    public Pair(K key, V value) {
        this.key = key;
        this.value = value;
    }

    public K getKey() {
        return key;
    }

    public V getValue() {
        return value;
    }
}