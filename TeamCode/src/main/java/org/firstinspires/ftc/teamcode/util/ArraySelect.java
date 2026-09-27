package org.firstinspires.ftc.teamcode.util;

/**
 * Non-looping list of objects with moveable index. Can not move past boundaries
 *
 * @param <T> Type of object inside the carousel
 */
public class ArraySelect<T> {
    /*
        Representation for array of size four
        ------------------------------------------------------------------------------
        |      value0      |     > value1 <    |      value2      |      value3      |
        ------------------------------------------------------------------------------
                                                                           ↑ last index -> can only move to previous
        currentIndex = 1 (2nd space)
        moveSelection(3) -> currentIndex = 3
     */

    private final T[] options;
    private int currentIndex = 0;

    /**
     * Creates a carousel selection of the generic type T
     *
     * @param options List of any type
     */
    public ArraySelect(T[] options) {
        this.options = options;
    }

    /**
     * @return Currently selected object inside the carousel
     */
    public T getSelected() {
        return options[currentIndex];
    }

    public int getSelectedIndex() {
        return currentIndex;
    }

    public T[] getAllOptions() {
        return options;
    }

    /**
     * Move the index of the carousel by 1. If the index ends out of bounds of the array, it will stay at the first or last item
     *
     * @return Returns the CarouselSelect object. Useful for chaining methods
     */
    public ArraySelect<T> next() {
        return moveSelection(1);
    }

    /**
     * Move the index of the carousel by 1. If the index ends out of bounds of the array, it will stay at the first or last item
     *
     * @return Returns the CarouselSelect object. Useful for chaining methods
     */
    public ArraySelect<T> previous() {
        return moveSelection(-1);
    }

    /**
     * Move the index of the selector by a specified amount. If the index ends out of bounds of the array, it will stay at the first or last item
     *
     * @param amount Amount to be added to the index
     * @return Returns the CarouselSelect object. Useful for chaining methods
     */
    public ArraySelect<T> moveSelection(int amount) {
        if (currentIndex + amount >= 0 && currentIndex + amount < options.length) {
            currentIndex += amount;
        } else if (currentIndex + amount < 0) {
            currentIndex = 0;
        } else if (currentIndex + amount >= options.length) {
            currentIndex = options.length - 1;
        }
        return this;
    }

    public void setSelected(int index) {
        currentIndex = index;
    }

    public void setSelected(T state){
        for(int i = 0; i < options.length; i++){
            if(options[i] == state) setSelected(i);
        }
    }
}
